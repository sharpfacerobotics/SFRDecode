package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Turns Limelight neural-detector results (robots seen by the camera) into field-frame obstacles
 * WITH velocity, by tracking detections across frames.
 *
 * How it works each call:
 *   1. If the latest Limelight frame is fresh AND came from {@link #DETECTOR_PIPELINE}, every
 *      detection above {@link #MIN_CONFIDENCE} is projected into field coordinates: bearing from
 *      tx (same {@link GoalLocator#TX_SIGN} convention as the goal math), range from the ty
 *      ground-plane intersection (camera height/pitch), falling back to the apparent-area model
 *      when the geometry is degenerate.
 *   2. Measurements are matched to existing tracks (nearest-neighbor inside {@link #GATE_RADIUS});
 *      matched tracks get EMA-smoothed position and finite-difference velocity, unmatched
 *      measurements start new tracks.
 *   3. Between detector frames (e.g. while the camera is on the AprilTag pipeline) tracks COAST on
 *      their last velocity, and are dropped after {@link #MAX_COAST_TIME} — stale ghosts never
 *      linger, and "camera busy elsewhere" degrades to "briefly predicted, then honest nothing".
 *
 * WHAT MUST BE CONFIGURED before this produces anything (all dashboard-tunable):
 *   - DETECTOR_PIPELINE: index of a neural-detector pipeline trained to see robots (-1 = off,
 *     returns no obstacles — the AutoPositioner then simply drives clean lines).
 *   - CAM_HEIGHT_IN / CAM_PITCH_DEG: measure the lens on the robot (used for ty->range).
 *   - RANGE_K / RANGE_EXP: fit the area fallback exactly like Vision.distanceCalc() was fit.
 *   - GoalLocator.TX_SIGN: one field check fixes the bearing sign for goal AND obstacles.
 *
 * Poses come from a PoseProvider so this class stays decoupled from Drivetrain/Follower.
 */
public class LimelightObstacleSource implements ObstacleSource {

    /** Supplies the robot's current field pose (e.g. () -> follower.getPose()). */
    public interface PoseProvider { Pose getPose(); }

    // --- Pipeline / acceptance ---
    /** Pipeline index of the robot-detector model. -1 disables (honestly reports nothing). */
    public static int    DETECTOR_PIPELINE = -1;
    public static double MIN_CONFIDENCE    = 0.5;
    public static double MAX_STALENESS_MS  = 250;

    // --- Range models ---
    /** Lens height above the floor, inches. MEASURE on the robot. */
    public static double CAM_HEIGHT_IN  = 10.0;
    /** Camera pitch, degrees, positive = tilted up. MEASURE on the robot. */
    public static double CAM_PITCH_DEG  = -10.0;
    /** Height of the aim point on an opponent robot (bounding-box center), inches. */
    public static double TARGET_HEIGHT_IN = 8.0;
    /** Area fallback: rangeInches = RANGE_K * area^RANGE_EXP. FIT to your camera. */
    public static double RANGE_K   = 120.0;
    public static double RANGE_EXP = -0.5;
    /** Camera offset forward of robot center, inches. */
    public static double CAM_FORWARD = 6.0;
    public static double MAX_RANGE = 200.0;   // reject absurd projections

    // --- Tracking ---
    public static double GATE_RADIUS    = 24.0;  // in, measurement-to-track association gate
    public static double POS_SMOOTHING  = 0.5;   // EMA blend toward the new position (0..1)
    public static double VEL_SMOOTHING  = 0.3;   // EMA blend toward the new velocity (0..1)
    public static double MAX_SPEED      = 80.0;  // in/s, caps association-error spikes
    public static double MAX_COAST_TIME = 0.75;  // s, drop tracks unseen this long
    public static double MIN_VEL_DT     = 0.02;  // s, min spacing for a velocity sample

    private static class Track {
        double x, y, vx, vy;
        double lastSeen;
        Track(double x, double y, double t) { this.x = x; this.y = y; this.lastSeen = t; }
    }

    private final Limelight3A limelight;
    private final PoseProvider poseProvider;
    private final ElapsedTime clock = new ElapsedTime();
    private final List<Track> tracks = new ArrayList<>();

    public LimelightObstacleSource(Limelight3A limelight, PoseProvider poseProvider) {
        this.limelight = limelight;
        this.poseProvider = poseProvider;
    }

    public void reset() { tracks.clear(); clock.reset(); }

    public int getTrackCount() { return tracks.size(); }

    @Override
    public List<Obstacle> getObstacles() {
        double now = clock.seconds();

        if (DETECTOR_PIPELINE >= 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && result.getPipelineIndex() == DETECTOR_PIPELINE
                    && result.getStaleness() <= MAX_STALENESS_MS) {
                ingest(result, now);
            }
        }

        // Prune tracks that have coasted too long, then emit predicted positions.
        List<Obstacle> out = new ArrayList<>();
        Iterator<Track> it = tracks.iterator();
        while (it.hasNext()) {
            Track t = it.next();
            double coast = now - t.lastSeen;
            if (coast > MAX_COAST_TIME) { it.remove(); continue; }
            out.add(new Obstacle(t.x + t.vx * coast, t.y + t.vy * coast,
                    t.vx, t.vy, FieldConfig.DEFAULT_OBSTACLE_RADIUS));
        }
        return out;
    }

    /** Project this frame's detections to field coords and fold them into the track list. */
    private void ingest(LLResult result, double now) {
        Pose robot = poseProvider.getPose();
        double camX = robot.getX() + CAM_FORWARD * Math.cos(robot.getHeading());
        double camY = robot.getY() + CAM_FORWARD * Math.sin(robot.getHeading());

        List<double[]> measurements = new ArrayList<>();
        for (LLResultTypes.DetectorResult det : result.getDetectorResults()) {
            if (det.getConfidence() < MIN_CONFIDENCE) continue;

            double range = estimateRange(det.getTargetYDegrees(), det.getTargetArea());
            if (range <= 0 || range > MAX_RANGE) continue;

            double bearing = robot.getHeading()
                    + GoalLocator.TX_SIGN * Math.toRadians(det.getTargetXDegrees());
            double ox = camX + range * Math.cos(bearing);
            double oy = camY + range * Math.sin(bearing);

            // A detection projected outside the field is a bad range/false positive — drop it.
            double lo = FieldConfig.FIELD_MIN - 6, hi = FieldConfig.FIELD_MIN + FieldConfig.FIELD_SIZE + 6;
            if (ox < lo || ox > hi || oy < lo || oy > hi) continue;

            measurements.add(new double[]{ox, oy});
        }

        boolean[] trackUsed = new boolean[tracks.size()];
        for (double[] m : measurements) {
            // Nearest unclaimed track inside the gate.
            int bestIdx = -1;
            double bestDist = GATE_RADIUS;
            for (int i = 0; i < tracks.size(); i++) {
                if (trackUsed[i]) continue;
                Track t = tracks.get(i);
                double d = Math.hypot(m[0] - t.x, m[1] - t.y);
                if (d < bestDist) { bestDist = d; bestIdx = i; }
            }

            if (bestIdx < 0) {
                tracks.add(new Track(m[0], m[1], now));   // new obstacle, velocity unknown yet
                continue;
            }

            Track t = tracks.get(bestIdx);
            trackUsed[bestIdx] = true;
            double dt = now - t.lastSeen;
            if (dt >= MIN_VEL_DT) {
                double rvx = Range.clip((m[0] - t.x) / dt, -MAX_SPEED, MAX_SPEED);
                double rvy = Range.clip((m[1] - t.y) / dt, -MAX_SPEED, MAX_SPEED);
                t.vx = t.vx + VEL_SMOOTHING * (rvx - t.vx);
                t.vy = t.vy + VEL_SMOOTHING * (rvy - t.vy);
            }
            t.x = t.x + POS_SMOOTHING * (m[0] - t.x);
            t.y = t.y + POS_SMOOTHING * (m[1] - t.y);
            t.lastSeen = now;
        }
    }

    /**
     * Range to the detection, inches. Primary: ty ground-plane intersection
     * (the same geometry as the classic distance-from-target formula). Fallback: apparent area.
     */
    private double estimateRange(double tyDegrees, double area) {
        double angle = Math.toRadians(CAM_PITCH_DEG + tyDegrees);
        double tan = Math.tan(angle);
        if (Math.abs(tan) > 0.02) {
            double range = (TARGET_HEIGHT_IN - CAM_HEIGHT_IN) / tan;
            if (range > 0 && range <= MAX_RANGE) return range;
        }
        // Degenerate geometry (target near the horizon) — fall back to size-based estimate.
        if (area > 0) return RANGE_K * Math.pow(area, RANGE_EXP);
        return -1;
    }
}
