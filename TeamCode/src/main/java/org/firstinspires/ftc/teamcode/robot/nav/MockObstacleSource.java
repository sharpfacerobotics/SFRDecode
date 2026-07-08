package org.firstinspires.ftc.teamcode.robot.nav;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Scripted moving obstacles for testing avoidance ON THE ROBOT without needing opponents.
 *
 * Each obstacle follows a deterministic path (line sweep / circle) so you can watch the
 * AutoPositioner bend its spline and dodge in real time on the field or in the dashboard.
 * Velocities are the analytic derivative of the path, so the reactive prediction is honest.
 *
 * This is a TEST source. For real matches, swap in a sensing-backed ObstacleSource.
 */
public class MockObstacleSource implements ObstacleSource {

    private final ElapsedTime clock = new ElapsedTime();
    private boolean enabled = true;

    public void setEnabled(boolean e) { enabled = e; }
    public void reset() { clock.reset(); }

    @Override
    public List<Obstacle> getObstacles() {
        List<Obstacle> out = new ArrayList<>();
        if (!enabled) return out;

        double t = clock.seconds();

        // Obstacle A: sweeps left<->right across mid-field at y=90, period 8s.
        double wA = 2 * Math.PI / 8.0;
        double xA = 72 + 45 * Math.sin(wA * t);
        double vxA = 45 * wA * Math.cos(wA * t);
        out.add(new Obstacle(xA, 90, vxA, 0, FieldConfig.DEFAULT_OBSTACLE_RADIUS));

        // Obstacle B: circles the field center, radius 30, period 10s.
        double wB = 2 * Math.PI / 10.0;
        double xB = 72 + 30 * Math.cos(wB * t);
        double yB = 72 + 30 * Math.sin(wB * t);
        double vxB = -30 * wB * Math.sin(wB * t);
        double vyB = 30 * wB * Math.cos(wB * t);
        out.add(new Obstacle(xB, yB, vxB, vyB, FieldConfig.DEFAULT_OBSTACLE_RADIUS));

        return out;
    }
}
