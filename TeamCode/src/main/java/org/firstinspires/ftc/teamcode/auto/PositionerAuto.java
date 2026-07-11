package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.nav.AutoPositioner;
import org.firstinspires.ftc.teamcode.robot.nav.GoalLocator;
import org.firstinspires.ftc.teamcode.robot.nav.LimelightObstacleSource;
import org.firstinspires.ftc.teamcode.robot.nav.MockObstacleSource;
import org.firstinspires.ftc.teamcode.robot.nav.NavGeometry;
import org.firstinspires.ftc.teamcode.robot.nav.ObstacleSource;

/**
 * Autonomous built on the AutoPositioner: instead of driving to a hardcoded shootPose, each
 * scoring leg navigates by live localization (Pinpoint + Limelight MT2 pre-start fix) to
 * whatever scoring pose is currently best — replanning its spline as obstacles move and
 * dodging fast movers reactively. Shooting and the intake cycle reuse the same state-machine
 * patterns (and timing values) as FarAuto so behavior on the launcher side is identical.
 *
 * Cycle: POSITION_TO_SCORE (AutoPositioner) → SHOOT (lock-on + 3 shots, FarAuto-style)
 *        → DRIVE_TO_INTAKE (obstacle-aware spline) → back to POSITION_TO_SCORE … → DONE.
 *
 * Subclasses supply the alliance goal, start pose, and intake pose (see
 * pedroAutoPositionerRed / pedroAutoPositionerBlue).
 *
 * Obstacle sensing is pluggable via ObstacleSource. The robot has no opponent-sensing hardware
 * yet, so the default source is a disabled MockObstacleSource (= no obstacles, drives clean
 * lines). Set USE_MOCK_OBSTACLES = true to demo avoidance against scripted movers.
 */
public abstract class PositionerAuto extends BaseAuto {

    // --- Tunables ---
    public static boolean USE_MOCK_OBSTACLES      = false;
    public static boolean USE_LIMELIGHT_OBSTACLES = false;
    public static double POSITION_TIMEOUT      = 6.0;  // s, shoot from wherever we are after this
    public static double INTAKE_ARRIVE_TIMEOUT = 3.0;  // s, mirrors FarAuto's intake-N leg
    public static int    INTAKE_CYCLES         = 2;    // intake trips after the preload volley

    // Shot timing — same values as FarAuto
    protected double launchTimeoutTime  = 2.0;
    protected double launchWaitTime     = 0.4;
    protected double launchTransferTime = 0.5;
    protected double launchShootTime    = 0.2;
    protected double launchResetTime    = 0.15;

    public enum AutoState { POSITION_TO_SCORE, SHOOT, DRIVE_TO_INTAKE, DONE }
    public enum LaunchState { WAITING, TRANSFER, SHOOT, RESET }

    protected AutoState   autoState   = AutoState.POSITION_TO_SCORE;
    protected LaunchState launchState = LaunchState.WAITING;

    protected MockObstacleSource mockObstacles;
    protected LimelightObstacleSource limelightObstacles;
    protected ObstacleSource obstacleSource;
    protected AutoPositioner autoPositioner;
    protected GoalLocator goalLocator;

    private final ElapsedTime launchTimer = new ElapsedTime();
    private boolean isTargetLocked = false;
    private int intakeTripsDone = 0;

    /** Alliance goal to score into (FieldConfig.RED_GOAL / BLUE_GOAL fallback; live-calibrated). */
    protected abstract Pose goalPose();

    /** Where to collect balls between volleys. */
    protected abstract Pose intakePose();

    protected void setAutoState(AutoState newState) {
        autoState = newState;
        pathTimer.resetTimer();
    }

    @Override
    protected void buildPaths() {
        // Paths are built live from the current pose each leg — nothing to prebuild.
    }

    @Override
    public void init() {
        super.init();
        vision.init();   // start the Limelight on the goal-tag pipeline

        mockObstacles = new MockObstacleSource();
        mockObstacles.setEnabled(USE_MOCK_OBSTACLES);
        if (USE_LIMELIGHT_OBSTACLES) {
            limelightObstacles = new LimelightObstacleSource(robot.limelight,
                    () -> follower.getPose());
            obstacleSource = limelightObstacles;
        } else {
            obstacleSource = mockObstacles;
        }

        goalLocator = new GoalLocator(goalPose());
        autoPositioner = new AutoPositioner(follower, obstacleSource, goalPose());
        autoPositioner.setGoalLocator(goalLocator);
    }

    /** Pre-start: refine the defined startPose with a live MT2 fix if a tag is visible. */
    @Override
    public void init_loop() {
        if (vision.setPose()) {
            follower.setPose(vision.startPose);
            telemetry.addData("Localized (MT2)", "(%.1f, %.1f)",
                    vision.startPose.getX(), vision.startPose.getY());
        } else {
            telemetry.addData("Localization", "no tag — using defined startPose");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        mockObstacles.reset();
        if (limelightObstacles != null) limelightObstacles.reset();
        setLaunchPower(launchPower);
        autoPositioner.engage();
        setAutoState(AutoState.POSITION_TO_SCORE);
    }

    @Override
    protected void statePathUpdate() {
        // Live goal calibration: whenever the goal tag is in view, refine where the goal is.
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            goalLocator.update(follower.getPose(), result.getTx(),
                    vision.distanceCalc(result.getTa()));
        }

        // Drive the positioner (no-op while disengaged).
        autoPositioner.update();

        switch (autoState) {
            case POSITION_TO_SCORE:
                setLaunchPower(launchPower);   // spin up while driving
                boolean timedOut = pathTimer.getElapsedTimeSeconds() > POSITION_TIMEOUT;
                if (autoPositioner.isReadyToShoot() || timedOut) {
                    if (timedOut) telemetry.addLine("WARNING: position timeout — shooting from here");
                    autoPositioner.disengage();   // freeze; lock-on takes over aiming
                    shotsFired = 0;
                    isTargetLocked = false;
                    launchState = LaunchState.WAITING;
                    launchTimer.reset();
                    setAutoState(AutoState.SHOOT);
                }
                break;

            case SHOOT:
                shootSequence();
                if (shotsFired >= 3) {
                    if (intakeTripsDone >= INTAKE_CYCLES) {
                        robot.intakeMotor.setPower(0);
                        robot.transferMotor.setPower(0);
                        setAutoState(AutoState.DONE);
                    } else {
                        robot.intakeMotor.setPower(1);
                        robot.transferMotor.setPower(-1);
                        followSplineTo(intakePose());
                        setAutoState(AutoState.DRIVE_TO_INTAKE);
                    }
                }
                break;

            case DRIVE_TO_INTAKE:
                // Same arrival condition as FarAuto's intake-N leg.
                if ((!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0)
                        || pathTimer.getElapsedTimeSeconds() > INTAKE_ARRIVE_TIMEOUT) {
                    intakeTripsDone++;
                    autoPositioner.engage();   // intake + transfer keep running on the way back
                    setAutoState(AutoState.POSITION_TO_SCORE);
                }
                break;

            case DONE:
                telemetry.addLine("Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No State Set");
                break;
        }

        telemetry.addData("Auto state", autoState);
        telemetry.addLine(autoPositioner.telemetry());
        telemetry.addData("Goal fix", goalLocator.hasLiveFix()
                ? String.format("LIVE (%d sightings) (%.1f, %.1f)", goalLocator.getSightings(),
                    goalLocator.getGoal().getX(), goalLocator.getGoal().getY())
                : "fallback (tag not seen yet)");
        telemetry.addData("Intake trips", intakeTripsDone + " / " + INTAKE_CYCLES);
    }

    /**
     * One obstacle-aware spline leg to a fixed target (used for the intake run): straight
     * BezierLine when the lane is clear, bowed BezierCurve around the blocker when it isn't.
     */
    private void followSplineTo(Pose target) {
        Pose robotPose = follower.getPose();
        Pose control = NavGeometry.avoidanceControlPoint(robotPose, target,
                obstacleSource.getObstacles());
        PathChain path = (control == null)
                ? follower.pathBuilder()
                    .addPath(new BezierLine(robotPose, target))
                    .setLinearHeadingInterpolation(robotPose.getHeading(), target.getHeading())
                    .build()
                : follower.pathBuilder()
                    .addPath(new BezierCurve(robotPose, control, target))
                    .setLinearHeadingInterpolation(robotPose.getHeading(), target.getHeading())
                    .build();
        follower.followPath(path, true);
    }

    // --- Shooting: same lock-on + shot machine pattern as FarAuto ---

    private void shootSequence() {
        if (!isTargetLocked) lockOn();

        if (pathTimer.getElapsedTimeSeconds() < spinUpTime) {
            robot.intakeMotor.setPower(0);
            robot.transferMotor.setPower(0);
            setLaunchPower(launchPower);
            launchTimer.reset();
            return;
        }
        if (shotsFired < 3) shootOneShot();
    }

    private void lockOn() {
        LLResult result = robot.limelight.getLatestResult();
        vision.setHasTarget(result != null && result.isValid());

        double correction = vision.getYaw(0.0, true, result) * 0.6;
        double turnPower = Range.clip(correction, -0.5, 0.5);
        robot.frontLeftDrive.setPower(-turnPower);
        robot.frontRightDrive.setPower(turnPower);
        robot.backLeftDrive.setPower(-turnPower);
        robot.backRightDrive.setPower(turnPower);

        if (vision.isLockedOn()) {
            isTargetLocked = true;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            follower.holdPoint(follower.getPose());   // freeze on the locked heading
            launchTimer.reset();
        }
    }

    private void shootOneShot() {
        if (launchTimer.seconds() > launchTimeoutTime) {
            launchState = LaunchState.WAITING;
            launchTimer.reset();
            telemetry.addLine("WARNING: Launch state timeout");
            return;
        }
        double elapsed = launchTimer.seconds();
        setLaunchPower(launchPower);
        switch (launchState) {
            case WAITING:
                telemetry.addLine("Waiting");
                if (elapsed > launchWaitTime) {
                    launchState = LaunchState.TRANSFER;
                    launchTimer.reset();
                }
                break;

            case TRANSFER:
                robot.transferMotor.setPower(1);
                telemetry.addLine("Shot " + (shotsFired + 1) + ": Firing");
                if (elapsed > launchTransferTime) {
                    launchState = LaunchState.SHOOT;
                    launchTimer.reset();
                }
                break;

            case SHOOT:
                robot.Trigger.setPosition(triggerShootPos);
                robot.transferMotor.setPower(0);
                if (elapsed > launchShootTime) {
                    launchState = LaunchState.RESET;
                    launchTimer.reset();
                }
                break;

            case RESET:
                robot.Trigger.setPosition(triggerStartPos);
                telemetry.addLine("Shot " + (shotsFired + 1) + ": Resetting");
                if (elapsed > launchResetTime) {
                    launchState = LaunchState.WAITING;
                    ++shotsFired;
                    launchTimer.reset();
                }
                break;
        }
    }
}
