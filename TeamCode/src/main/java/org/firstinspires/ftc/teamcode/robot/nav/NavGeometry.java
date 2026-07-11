package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.geometry.Pose;

import java.util.List;

/**
 * Pure geometry for the auto-positioner. No FTC SDK, no follower, no side effects — every method
 * takes numbers/poses in and returns numbers/poses out, so it can be reasoned about and unit-tested.
 *
 * Responsibilities:
 *   1. bestScoringPose(): pick a field pose we can score from (in range, clear shot, in bounds,
 *      clear of obstacles), preferring the least travel from where we are now.
 *   2. avoidanceControlPoint(): if the straight line to that pose is blocked, produce a Bezier
 *      control point that bows the spline around the blocking obstacle.
 *   3. imminentThreat() / escapePose(): reactive layer for a fast mover we can't spline around in time.
 */
public final class NavGeometry {

    private NavGeometry() {}

    // ---------------------------------------------------------------------------------------------
    // 1. Scoring pose selection
    // ---------------------------------------------------------------------------------------------

    /**
     * Choose the best pose to score from.
     *
     * Samples a set of candidate poses on rings around the goal (across the shooting-distance band),
     * rejects any that are out of bounds, blocked from the goal, or too close to an obstacle, and
     * returns the survivor closest to the robot. Returns null only if nothing is valid.
     */
    public static Pose bestScoringPose(Pose robot, Pose goal, List<Obstacle> obstacles) {
        double clearance = FieldConfig.ROBOT_RADIUS + FieldConfig.SAFETY_MARGIN;

        Pose best = null;
        double bestCost = Double.MAX_VALUE;
        Pose bestBlockedFallback = null;
        double bestBlockedCost = Double.MAX_VALUE;

        double minR = FieldConfig.MIN_SHOOT_DISTANCE;
        double maxR = FieldConfig.MAX_SHOOT_DISTANCE;

        // A few radii across the band; many angles around the goal.
        for (int ri = 0; ri <= 4; ri++) {
            double r = minR + (maxR - minR) * (ri / 4.0);
            for (int deg = 0; deg < 360; deg += 5) {
                double a = Math.toRadians(deg);
                double x = goal.getX() + r * Math.cos(a);
                double y = goal.getY() + r * Math.sin(a);

                if (!inBounds(x, y)) continue;

                double heading = FieldConfig.headingToFaceGoal(x, y, goal);
                Pose cand = new Pose(x, y, heading);

                double travel = dist(robot.getX(), robot.getY(), x, y);
                // Small penalty for having to rotate a lot on arrival.
                double turn = Math.abs(wrap(heading - robot.getHeading()));
                double cost = travel + turn * 6.0;

                boolean poseClear = minObstacleClearance(x, y, obstacles) >= clearance;
                boolean shotClear = !segmentBlocked(x, y, goal.getX(), goal.getY(), obstacles, 0);

                if (poseClear && shotClear) {
                    if (cost < bestCost) { bestCost = cost; best = cand; }
                } else if (poseClear) {
                    // Valid standing spot but shot momentarily blocked — keep as fallback.
                    if (cost < bestBlockedCost) { bestBlockedCost = cost; bestBlockedFallback = cand; }
                }
            }
        }

        return best != null ? best : bestBlockedFallback;
    }

    // ---------------------------------------------------------------------------------------------
    // 2. Spline avoidance
    // ---------------------------------------------------------------------------------------------

    /**
     * If an obstacle blocks the straight path from start to end, return a control point that pushes
     * a quadratic Bezier around it (on the open side). Returns null if the straight line is clear,
     * in which case the caller should use a BezierLine.
     */
    public static Pose avoidanceControlPoint(Pose start, Pose end, List<Obstacle> obstacles) {
        double need = FieldConfig.ROBOT_RADIUS + FieldConfig.SAFETY_MARGIN;

        Obstacle worst = null;
        double worstIntrusion = 0;
        for (Obstacle o : obstacles) {
            double d = pointToSegment(o.x, o.y, start.getX(), start.getY(), end.getX(), end.getY());
            double intrusion = (o.radius + need) - d;   // >0 means it cuts into our corridor
            if (intrusion > worstIntrusion) { worstIntrusion = intrusion; worst = o; }
        }
        if (worst == null) return null;   // path is clear, use a straight line

        // Closest point on the segment to the obstacle center.
        double[] c = closestPointOnSegment(worst.x, worst.y,
                start.getX(), start.getY(), end.getX(), end.getY());

        // Push direction: away from the obstacle. If the obstacle sits exactly on the line,
        // push perpendicular to the segment instead.
        double dx = c[0] - worst.x, dy = c[1] - worst.y;
        double len = Math.hypot(dx, dy);
        if (len < 1e-6) {
            double sx = end.getX() - start.getX(), sy = end.getY() - start.getY();
            double sl = Math.hypot(sx, sy);
            dx = -sy / sl; dy = sx / sl;   // left-hand normal
        } else {
            dx /= len; dy /= len;
        }

        // Bow the curve out far enough that its midpoint (~half the control offset) clears the
        // obstacle. Overshoot a little; the path is re-evaluated continuously anyway.
        double push = (worst.radius + need) * 2.2;
        double cx = clampAxis(c[0] + dx * push);
        double cy = clampAxis(c[1] + dy * push);
        return new Pose(cx, cy, end.getHeading());
    }

    // ---------------------------------------------------------------------------------------------
    // 3. Reactive layer (fast movers)
    // ---------------------------------------------------------------------------------------------

    /**
     * Is any obstacle predicted to come within the hard-stop radius of the robot within `lookahead`
     * seconds? Uses constant-velocity prediction. Returns the threatening obstacle or null.
     */
    public static Obstacle imminentThreat(Pose robot, List<Obstacle> obstacles,
                                          double lookahead, double hardRadius) {
        Obstacle threat = null;
        double soonest = Double.MAX_VALUE;
        for (Obstacle o : obstacles) {
            // sample the predicted separation over the lookahead window
            for (double t = 0; t <= lookahead; t += 0.1) {
                double d = dist(robot.getX(), robot.getY(), o.predictedX(t), o.predictedY(t));
                double trigger = hardRadius + o.radius + FieldConfig.ROBOT_RADIUS;
                if (d < trigger && t < soonest) { soonest = t; threat = o; break; }
            }
        }
        return threat;
    }

    /**
     * Pick a short evasive pose that moves the robot laterally away from the threat's approach,
     * staying in bounds and keeping the launcher pointed at the goal for fast re-acquisition.
     */
    public static Pose escapePose(Pose robot, Obstacle threat, Pose goal,
                                  List<Obstacle> obstacles, double escapeDist) {
        // Direction from threat to robot = "straight away".
        double ax = robot.getX() - threat.x, ay = robot.getY() - threat.y;
        double al = Math.hypot(ax, ay);
        if (al < 1e-6) { ax = 1; ay = 0; al = 1; }
        ax /= al; ay /= al;

        // Two perpendicular escape options (sidestep is usually better than backing straight up).
        double[][] dirs = {
                { ax, ay },        // directly away
                { -ay, ax },       // sidestep left
                { ay, -ax }        // sidestep right
        };

        Pose best = null;
        double bestClear = -1;
        for (double[] d : dirs) {
            double x = clampInBounds(robot.getX() + d[0] * escapeDist);
            double y = clampInBounds(robot.getY() + d[1] * escapeDist);
            double clear = minObstacleClearance(x, y, obstacles);
            if (clear > bestClear) {
                bestClear = clear;
                best = new Pose(x, y, FieldConfig.headingToFaceGoal(x, y, goal));
            }
        }
        return best;
    }

    // ---------------------------------------------------------------------------------------------
    // Shared helpers
    // ---------------------------------------------------------------------------------------------

    /** True if the segment a->b passes within (obstacle.radius + extra + robot) of any obstacle. */
    public static boolean segmentBlocked(double ax, double ay, double bx, double by,
                                         List<Obstacle> obstacles, double extra) {
        for (Obstacle o : obstacles) {
            double d = pointToSegment(o.x, o.y, ax, ay, bx, by);
            if (d < o.radius + extra + FieldConfig.ROBOT_RADIUS) return true;
        }
        return false;
    }

    public static double minObstacleClearance(double x, double y, List<Obstacle> obstacles) {
        double min = Double.MAX_VALUE;
        for (Obstacle o : obstacles) {
            min = Math.min(min, dist(x, y, o.x, o.y) - o.radius);
        }
        return min;
    }

    public static boolean inBounds(double x, double y) {
        double lo = FieldConfig.FIELD_MIN + FieldConfig.WALL_MARGIN;
        double hi = FieldConfig.FIELD_MIN + FieldConfig.FIELD_SIZE - FieldConfig.WALL_MARGIN;
        return x >= lo && x <= hi && y >= lo && y <= hi;
    }

    private static double clampInBounds(double v) {
        double lo = FieldConfig.FIELD_MIN + FieldConfig.WALL_MARGIN;
        double hi = FieldConfig.FIELD_MIN + FieldConfig.FIELD_SIZE - FieldConfig.WALL_MARGIN;
        return Math.max(lo, Math.min(hi, v));
    }

    /** Clamp to the physical field (control points may sit outside the wall margin). */
    private static double clampAxis(double v) {
        double lo = FieldConfig.FIELD_MIN + 2.0;
        double hi = FieldConfig.FIELD_MIN + FieldConfig.FIELD_SIZE - 2.0;
        return Math.max(lo, Math.min(hi, v));
    }

    public static double dist(double x1, double y1, double x2, double y2) {
        return Math.hypot(x1 - x2, y1 - y2);
    }

    /** Shortest distance from point p to segment a->b. */
    public static double pointToSegment(double px, double py,
                                        double ax, double ay, double bx, double by) {
        double[] c = closestPointOnSegment(px, py, ax, ay, bx, by);
        return dist(px, py, c[0], c[1]);
    }

    public static double[] closestPointOnSegment(double px, double py,
                                                 double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay;
        double len2 = dx * dx + dy * dy;
        if (len2 < 1e-9) return new double[]{ ax, ay };
        double t = ((px - ax) * dx + (py - ay) * dy) / len2;
        t = Math.max(0, Math.min(1, t));
        return new double[]{ ax + t * dx, ay + t * dy };
    }

    /** Wrap an angle to [-PI, PI]. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
