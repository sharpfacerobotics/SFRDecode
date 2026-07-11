package org.firstinspires.ftc.teamcode.robot.nav;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Auto-positioner: continuously drives the robot to a pose it can score from, using live
 * localization, a spline path that re-plans as things move, and a reactive layer that dodges
 * fast movers.
 *
 * How it plugs in (mirrors how CompTeleop already uses the follower):
 *   - opmode owns the {@link Follower} (from Drivetrain) and MUST still call follower.update()
 *     every loop — that's what actually moves the robot.
 *   - opmode calls {@link #engage()} once to start, {@link #update()} every loop, and reads
 *     {@link #isReadyToShoot()} to know when to fire.
 *   - {@link #disengage()} hands control back (breaks the current path).
 *
 * Priority each loop: (1) dodge an imminent collision, else (2) hold if already settled on a
 * good scoring pose, else (3) follow / re-plan a spline toward the best scoring pose.
 */
public class AutoPositioner {

    public enum State { DISENGAGED, POSITIONING, AVOIDING, EVADING, SETTLED, STUCK }

    private final Follower follower;
    private final ObstacleSource obstacleSource;
    private Pose goal;
    private GoalLocator goalLocator;   // optional: live goal from vision, overrides the fixed goal

    // --- Tuning ---
    public static double POS_TOLERANCE   = 2.5;   // in, "close enough" to the scoring pose
    public static double HEAD_TOLERANCE  = Math.toRadians(4);
    public static double REPLAN_PERIOD   = 1.2;   // s, background re-plan cadence (avoids jitter)
    public static double REPLAN_POSE_DELTA = 8.0; // in, re-plan if the chosen scoring pose shifts this far
    public static double LOOKAHEAD       = 0.6;   // s, collision prediction window
    public static double HARD_RADIUS     = 8.0;   // in, extra buffer that triggers a reactive dodge
    public static double ESCAPE_DIST     = 16.0;  // in, how far a dodge moves us
    public static double EVADE_MIN_HOLD  = 0.25;  // s, minimum time to commit to a dodge (anti-thrash)

    // --- State ---
    private State state = State.DISENGAGED;
    private Pose plannedTarget = null;
    private Pose currentScoringPose = null;
    private boolean holdingScoring = false;
    private final ElapsedTime replanTimer = new ElapsedTime();
    private final ElapsedTime evadeTimer = new ElapsedTime();
    private int replanCount = 0;
    private int avoidCount = 0;

    public AutoPositioner(Follower follower, ObstacleSource obstacleSource, Pose goal) {
        this.follower = follower;
        this.obstacleSource = obstacleSource;
        this.goal = goal;
    }

    public void setGoal(Pose goal) { this.goal = goal; }

    /** Optionally let a GoalLocator supply the goal live from vision instead of a fixed pose. */
    public void setGoalLocator(GoalLocator locator) { this.goalLocator = locator; }

    /** Begin auto-positioning from wherever the robot currently is. */
    public void engage() {
        state = State.POSITIONING;
        plannedTarget = null;
        holdingScoring = false;
        replanTimer.reset();
        planTo(follower.getPose(), obstacleSource.getObstacles());
    }

    /** Stop auto-positioning and release the follower. */
    public void disengage() {
        if (state != State.DISENGAGED) follower.breakFollowing();
        state = State.DISENGAGED;
        holdingScoring = false;
    }

    /** Call every loop while engaged (opmode still calls follower.update() separately). */
    public void update() {
        if (state == State.DISENGAGED) return;

        // If a live goal fix is available, use it (falls back to the fixed goal internally).
        if (goalLocator != null) goal = goalLocator.getGoal();

        Pose robot = follower.getPose();
        List<Obstacle> obs = obstacleSource.getObstacles();

        // (1) Reactive safety — a fast mover we can't spline around in time.
        Obstacle threat = NavGeometry.imminentThreat(robot, obs, LOOKAHEAD, HARD_RADIUS);
        if (threat != null) {
            Pose escape = NavGeometry.escapePose(robot, threat, goal, obs, ESCAPE_DIST);
            if (escape != null) {
                follower.holdPoint(escape);
                if (state != State.EVADING) evadeTimer.reset();
                state = State.EVADING;
                holdingScoring = false;
                plannedTarget = null;   // force a fresh plan once clear
                return;
            }
        }
        if (state == State.EVADING && evadeTimer.seconds() < EVADE_MIN_HOLD) {
            return;   // commit to the dodge briefly so we don't oscillate
        }

        // (2) Pick where to score from.
        Pose scoring = NavGeometry.bestScoringPose(robot, goal, obs);
        if (scoring == null) {
            state = State.STUCK;    // nowhere valid right now; hold position, keep re-evaluating
            return;
        }
        currentScoringPose = scoring;

        // (3) Already there with a clear shot? Hold and report ready.
        double posErr = NavGeometry.dist(robot.getX(), robot.getY(), scoring.getX(), scoring.getY());
        double headErr = Math.abs(NavGeometry.wrap(robot.getHeading() - scoring.getHeading()));
        boolean shotClear = !NavGeometry.segmentBlocked(
                robot.getX(), robot.getY(), goal.getX(), goal.getY(), obs, FieldConfig.SAFETY_MARGIN);

        if (posErr < POS_TOLERANCE && headErr < HEAD_TOLERANCE && shotClear) {
            if (!holdingScoring) { follower.holdPoint(scoring); holdingScoring = true; }
            state = State.SETTLED;
            return;
        }

        // (4) (Re)plan the spline when the target moved or the cadence timer elapsed.
        //
        // NOTE: we deliberately do NOT replan just because the straight line robot->target is
        // "blocked" — once avoidance is active we're intentionally following a curve around that
        // exact obstacle, so the straight line is SUPPOSED to stay blocked until we arrive. Firing
        // a replan on that condition was self-defeating: it fired almost every loop, restarting a
        // fresh curve from the current (barely-progressed) pose each time, so the follower never
        // accumulated real velocity and the robot stalled just short of the target. A moved/new
        // obstacle is instead picked up on the next cadence tick (planTo() recomputes the bow),
        // and anything urgent is caught immediately by the reactive EVADING layer above.
        boolean targetMoved = plannedTarget == null
                || NavGeometry.dist(plannedTarget.getX(), plannedTarget.getY(),
                                    scoring.getX(), scoring.getY()) > REPLAN_POSE_DELTA;
        boolean cadence = replanTimer.seconds() > REPLAN_PERIOD;

        if (targetMoved || cadence || holdingScoring) {
            planTo(robot, obs);
            replanTimer.reset();
        }
    }

    /** Build and follow a spline from the current pose to the current scoring pose. */
    private void planTo(Pose robot, List<Obstacle> obs) {
        Pose scoring = (currentScoringPose != null)
                ? currentScoringPose
                : NavGeometry.bestScoringPose(robot, goal, obs);
        if (scoring == null) { state = State.STUCK; return; }

        Pose control = NavGeometry.avoidanceControlPoint(robot, scoring, obs);
        PathChain path;
        if (control == null) {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(robot, scoring))
                    .setLinearHeadingInterpolation(robot.getHeading(), scoring.getHeading())
                    .build();
            state = State.POSITIONING;
        } else {
            path = follower.pathBuilder()
                    .addPath(new BezierCurve(robot, control, scoring))
                    .setLinearHeadingInterpolation(robot.getHeading(), scoring.getHeading())
                    .build();
            state = State.AVOIDING;
            avoidCount++;
        }
        follower.followPath(path, true);
        plannedTarget = scoring;
        holdingScoring = false;
        replanCount++;
    }

    // --- Reporting ---
    public boolean isReadyToShoot() { return state == State.SETTLED; }
    public boolean isAvoiding()     { return state == State.AVOIDING || state == State.EVADING; }
    public State getState()         { return state; }
    public Pose getScoringPose()    { return currentScoringPose; }
    public int getReplanCount()     { return replanCount; }
    public int getAvoidCount()      { return avoidCount; }

    public String telemetry() {
        Pose sp = currentScoringPose;
        return String.format("AutoPositioner[%s] target=%s replans=%d dodges=%d",
                state,
                sp == null ? "none" : String.format("(%.1f,%.1f@%.0f)",
                        sp.getX(), sp.getY(), Math.toDegrees(sp.getHeading())),
                replanCount, avoidCount);
    }
}
