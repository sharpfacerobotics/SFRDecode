package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns Limelight neural-detector results (robots seen by the camera) into field-frame obstacles.
 *
 * ================================ READ THIS — IT IS A STUB ================================
 * This only produces real obstacles if:
 *   1. Your Limelight is running a NEURAL DETECTOR pipeline trained to detect robots, AND
 *   2. you switch to that pipeline (this camera is otherwise on the AprilTag pipeline for scoring),
 *   3. you calibrate the bearing->range projection below for your camera mount + lens.
 *
 * A detector gives you an angle to the target (tx/ty) and a bounding-box size, NOT a distance.
 * Range from a single camera is an ESTIMATE (bigger box = closer). The projection below is a
 * placeholder using apparent size; you must fit RANGE_K / RANGE_EXP to your setup, exactly like
 * Vision.distanceCalc() was fit for the goal. Until then this returns an empty list, which the
 * AutoPositioner correctly treats as "I can't see obstacles" (no false confidence).
 *
 * Poses are provided by a PoseProvider so this class stays decoupled from Drivetrain/Follower.
 * =========================================================================================
 */
public class LimelightObstacleSource implements ObstacleSource {

    /** Supplies the robot's current field pose (e.g. () -> follower.getPose()). */
    public interface PoseProvider { Pose getPose(); }

    private final Limelight3A limelight;
    private final PoseProvider poseProvider;

    /** Pipeline index of the robot-detector model. Set to your trained pipeline; -1 disables. */
    public static int DETECTOR_PIPELINE = -1;

    /** Range estimate: distanceInches = RANGE_K * area^RANGE_EXP. FIT THESE to your camera. */
    public static double RANGE_K   = 120.0;
    public static double RANGE_EXP = -0.5;

    /** Horizontal camera offset from robot center, inches (forward mount = 0 lateral). */
    public static double CAM_FORWARD = 6.0;

    public LimelightObstacleSource(Limelight3A limelight, PoseProvider poseProvider) {
        this.limelight = limelight;
        this.poseProvider = poseProvider;
    }

    @Override
    public List<Obstacle> getObstacles() {
        List<Obstacle> out = new ArrayList<>();
        if (DETECTOR_PIPELINE < 0) return out;   // not configured -> honestly report nothing

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return out;

        Pose robot = poseProvider.getPose();

        for (LLResultTypes.DetectorResult det : result.getDetectorResults()) {
            double tx = det.getTargetXDegrees();          // bearing, camera frame
            double area = det.getTargetArea();            // % of image; proxy for range
            if (area <= 0) continue;

            double range = RANGE_K * Math.pow(area, RANGE_EXP);   // ESTIMATE — must be tuned

            // Project bearing+range from the robot's heading into field coords.
            double worldAngle = robot.getHeading() + Math.toRadians(tx);
            double camX = robot.getX() + CAM_FORWARD * Math.cos(robot.getHeading());
            double camY = robot.getY() + CAM_FORWARD * Math.sin(robot.getHeading());
            double ox = camX + range * Math.cos(worldAngle);
            double oy = camY + range * Math.sin(worldAngle);

            // Single-frame detection gives no velocity; leave it 0 (reactive layer still works on
            // proximity). Track frame-to-frame later to estimate vx/vy for prediction.
            out.add(new Obstacle(ox, oy, 0, 0, FieldConfig.DEFAULT_OBSTACLE_RADIUS));
        }
        return out;
    }
}
