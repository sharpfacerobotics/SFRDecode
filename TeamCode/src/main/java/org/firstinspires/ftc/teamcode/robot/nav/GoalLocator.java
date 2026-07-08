package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.geometry.Pose;

/**
 * Figures out WHERE the goal is on the field, live, from the Limelight — instead of trusting a
 * hardcoded coordinate. The Limelight already tracks the goal AprilTag, so every time the goal is
 * visible we can place it: it's `range` inches away, at bearing `tx` off the robot's heading, from
 * the robot's known field pose (botpose_MT2 / Pinpoint).
 *
 *     goal = robotXY + range * (cos(heading - tx), sin(heading - tx))
 *
 * Estimates are blended with an exponential moving average so a noisy frame doesn't jerk the target.
 * Uses ONLY quantities already produced in this codebase (follower pose, result.getTx(),
 * Vision.getDistance() from ta), so there are no unproven vision APIs here.
 *
 * If the goal has never been seen, getGoal() falls back to the FieldConfig default so the
 * positioner still has something to aim at.
 */
public class GoalLocator {

    /**
     * Sign of tx in the field frame. Limelight tx>0 means the tag is to the RIGHT of the crosshair;
     * with heading CCW-positive that is heading - tx. If your derived goal lands on the wrong side,
     * flip this to +1. (Same "verify the sign at a team meeting" caveat as Vision.)
     */
    public static double TX_SIGN = -1.0;

    /** Blend factor for the moving average (0..1): higher = trust the newest frame more. */
    public static double SMOOTHING = 0.2;

    /** Ignore obviously-bad range readings (inches). */
    public static double MIN_VALID_RANGE = 20.0;
    public static double MAX_VALID_RANGE = 200.0;

    private final Pose fallback;
    private Double goalX = null;
    private Double goalY = null;
    private int sightings = 0;

    public GoalLocator(Pose fallbackGoal) {
        this.fallback = fallbackGoal;
    }

    /**
     * Feed one frame in which the goal tag is visible.
     * @param robot       current robot field pose (follower.getPose())
     * @param txDegrees   Limelight tx (horizontal angle to tag, degrees)
     * @param rangeInches distance to the goal (Vision.getDistance())
     */
    public void update(Pose robot, double txDegrees, double rangeInches) {
        if (rangeInches < MIN_VALID_RANGE || rangeInches > MAX_VALID_RANGE) return;

        double bearing = robot.getHeading() + TX_SIGN * Math.toRadians(txDegrees);
        double ex = robot.getX() + rangeInches * Math.cos(bearing);
        double ey = robot.getY() + rangeInches * Math.sin(bearing);

        if (goalX == null) {
            goalX = ex; goalY = ey;
        } else {
            goalX = goalX + SMOOTHING * (ex - goalX);
            goalY = goalY + SMOOTHING * (ey - goalY);
        }
        sightings++;
    }

    /** Best current estimate of the goal (live if ever seen, else the FieldConfig fallback). */
    public Pose getGoal() {
        if (goalX == null) return fallback;
        return new Pose(goalX, goalY, 0);
    }

    public boolean hasLiveFix() { return goalX != null; }
    public int getSightings()   { return sightings; }
}
