package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.geometry.Pose;

/**
 * Central field + robot tuning for the auto-positioner.
 *
 * Coordinate frame matches the rest of this codebase (Pedro Pathing):
 *   - inches, origin at the field's bottom-left corner, field is 144" x 144"
 *   - field center is (72, 72) — see Vision.limelightToPedroPose()
 *   - heading in radians, 0 = +X, CCW positive
 *
 * NOTE ON GOAL COORDINATES:
 *   No explicit goal coordinate was stored anywhere in the repo, so BLUE_GOAL below is
 *   *derived* from the known-good CLOSE_BLUE_SHOOT_POSE (58.78, 84.27 @ 135deg): a ray from
 *   that pose along 135deg lands ~67" away in the top-left corner region, which is inside the
 *   launcher's 50-140" range. VERIFY this against your AprilTag field map and adjust —
 *   the solver's quality is only as good as this number.
 */
public final class FieldConfig {

    private FieldConfig() {}

    // --- Field geometry ---
    public static final double FIELD_SIZE = 144.0;
    /**
     * Field-frame coordinate of the field's own (0,0) origin corner, in this class's coordinate
     * system. 0 for the real robot (bottom-left-origin Pedro convention: field spans [0,144]).
     * Some simulators (e.g. virtual_robot) use a center-origin frame instead: set this to
     * -FIELD_SIZE/2 so the field spans [-72,72] and NavGeometry's bounds math still works unchanged.
     */
    public static double FIELD_MIN = 0.0;
    /** Keep the robot center at least this far from any wall when picking poses. */
    public static double WALL_MARGIN = 10.0;

    // --- Goal locations (point we score into). Heading is unused. ---
    public static Pose BLUE_GOAL = new Pose(12.0, 132.0, 0);
    public static Pose RED_GOAL  = mirror(BLUE_GOAL);

    // --- Launcher effective range (mirrors Launcher.MIN/MAX_DISTANCE) ---
    public static double MIN_SHOOT_DISTANCE = 55.0;   // a touch inside Launcher.MIN_DISTANCE for margin
    public static double MAX_SHOOT_DISTANCE = 135.0;  // a touch inside Launcher.MAX_DISTANCE for margin

    /**
     * Offset between "robot heading" and "direction the launcher fires".
     * 0 => the robot's +X face fires at the goal. If your launcher fires out the back, use PI.
     * If you have the always-on turret, heading matters less and this can stay 0.
     */
    public static double LAUNCH_HEADING_OFFSET = 0.0;

    // --- Robot / obstacle sizing (inches). Used for clearance + collision math. ---
    public static double ROBOT_RADIUS = 9.0;          // ~18" bot, treated as a circle
    public static double DEFAULT_OBSTACLE_RADIUS = 9.0;
    public static double SAFETY_MARGIN = 4.0;         // extra buffer added to every clearance test

    /** Mirror a pose across field center for the opposite alliance. */
    public static Pose mirror(Pose p) {
        double far = 2 * FIELD_MIN + FIELD_SIZE;
        return new Pose(far - p.getX(), far - p.getY(), p.getHeading() + Math.PI);
    }

    /** Heading that points the launcher at the goal from the given (x, y). */
    public static double headingToFaceGoal(double x, double y, Pose goal) {
        double bearing = Math.atan2(goal.getY() - y, goal.getX() - x);
        return bearing + LAUNCH_HEADING_OFFSET;
    }
}
