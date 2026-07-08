package org.firstinspires.ftc.teamcode.robot.nav;

import java.util.List;

/**
 * Anything that can tell the AutoPositioner where the moving obstacles are, in the field
 * frame (Pedro inches). This is the seam between "how you sense the field" and "how you avoid".
 *
 * Implementations in this repo:
 *   - MockObstacleSource      : scripted movers, for testing avoidance on the robot with no opponents
 *   - LimelightObstacleSource : projects Limelight neural-detector results into field coords (STUB)
 *
 * IMPORTANT: FTC gives the robot NO external feed of where opponents are. Every implementation
 * here can only report obstacles it can actually sense. An empty list means "I see nothing",
 * which the AutoPositioner treats as "no avoidance needed" — so a blind source == no avoidance.
 */
public interface ObstacleSource {

    /** @return current best estimate of obstacles, field frame. Never null; empty if none seen. */
    List<Obstacle> getObstacles();
}
