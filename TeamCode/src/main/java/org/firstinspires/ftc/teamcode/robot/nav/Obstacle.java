package org.firstinspires.ftc.teamcode.robot.nav;

/**
 * A single moving obstacle in the field frame (inches, Pedro coords).
 * Position is the obstacle's center; velocity is inches/second.
 *
 * This is intentionally a plain data holder so the avoidance math (NavGeometry) can be
 * unit-tested without any FTC SDK types, and so any sensing source (mock, Limelight
 * detector, distance sensors) can produce the same thing.
 */
public class Obstacle {

    public final double x;
    public final double y;
    public final double vx;
    public final double vy;
    public final double radius;

    public Obstacle(double x, double y, double vx, double vy, double radius) {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.radius = radius;
    }

    public Obstacle(double x, double y) {
        this(x, y, 0, 0, FieldConfig.DEFAULT_OBSTACLE_RADIUS);
    }

    /** Predicted center X after dt seconds, assuming constant velocity. */
    public double predictedX(double dt) { return x + vx * dt; }

    /** Predicted center Y after dt seconds, assuming constant velocity. */
    public double predictedY(double dt) { return y + vy * dt; }

    public double speed() { return Math.hypot(vx, vy); }
}
