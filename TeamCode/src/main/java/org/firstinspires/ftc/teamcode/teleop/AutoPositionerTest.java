package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.nav.AutoPositioner;
import org.firstinspires.ftc.teamcode.robot.nav.FieldConfig;
import org.firstinspires.ftc.teamcode.robot.nav.GoalLocator;
import org.firstinspires.ftc.teamcode.robot.nav.MockObstacleSource;
import org.firstinspires.ftc.teamcode.robot.nav.Obstacle;

import java.util.List;

/**
 * Demonstrates + tests the auto-positioner on the real robot using SCRIPTED moving obstacles
 * (MockObstacleSource) so you can see spline avoidance and reactive dodging with no opponents.
 *
 * Controls (gamepad1):
 *   A            : engage auto-positioning (drive to a scoring pose, avoiding movers)
 *   B            : disengage (hand control back)
 *   X            : toggle the mock obstacles on/off (compare paths with vs. without avoidance)
 *   right trigger: fire launcher (normal Launcher behavior)
 *
 * Watch the telemetry / FTC dashboard: state, chosen scoring pose, live obstacle positions,
 * re-plan count, and dodge count. When state == SETTLED the robot has a clear shot.
 *
 * This is BLUE-alliance by default (FieldConfig.BLUE_GOAL). For red, setGoal(FieldConfig.RED_GOAL).
 */
@TeleOp(name = "AutoPositioner Test (mock obstacles)", group = "nav")
public class AutoPositionerTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private Drivetrain drivetrain;
    private Launcher launcher;
    private Vision vision;

    private MockObstacleSource mockObstacles;
    private AutoPositioner autoPositioner;
    private GoalLocator goalLocator;

    private boolean prevA, prevB, prevX;
    private boolean mockEnabled = true;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        launcher = new Launcher(robot);
        launcher.init();

        drivetrain = new Drivetrain(robot);
        drivetrain.init(hardwareMap);

        vision = new Vision(robot);
        vision.init();

        mockObstacles = new MockObstacleSource();
        goalLocator = new GoalLocator(FieldConfig.BLUE_GOAL);
        autoPositioner = new AutoPositioner(drivetrain.follower, mockObstacles, FieldConfig.BLUE_GOAL);
        autoPositioner.setGoalLocator(goalLocator);   // self-calibrate the goal from the Limelight

        // Give the follower a valid starting pose. Prefer a live AprilTag fix; fall back to center.
        drivetrain.follower.setStartingPose(new Pose(72, 72, 0));

        telemetry.addLine("Init done. Aim at an AprilTag to localize, then press START.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.update();
            if (vision.setPose()) {
                drivetrain.follower.setPose(vision.startPose);
                telemetry.addData("Localized", "%.1f, %.1f", vision.startPose.getX(), vision.startPose.getY());
            } else {
                telemetry.addLine("No tag visible yet (will start from field center if none).");
            }
            telemetry.update();
        }

        waitForStart();
        mockObstacles.reset();

        while (opModeIsActive()) {
            // The follower must be updated every loop for the robot to actually move.
            drivetrain.follower.update();
            LLResult result = vision.update();
            double distance = vision.getDistance();
            double voltage = robot.myControlHubVoltageSensor.getVoltage();

            // Self-calibrate the goal position whenever the goal tag is visible.
            if (result != null && result.isValid() && vision.hasTarget()) {
                goalLocator.update(drivetrain.follower.getPose(), result.getTx(), distance);
            }

            // --- Edge-triggered buttons ---
            if (gamepad1.a && !prevA) autoPositioner.engage();
            if (gamepad1.b && !prevB) autoPositioner.disengage();
            if (gamepad1.x && !prevX) toggleMock();
            prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x;

            // --- Drive the auto-positioner ---
            autoPositioner.update();

            // --- Launcher stays available (fire manually with right trigger) ---
            launcher.update(gamepad1.right_trigger, false, vision.hasTarget(), distance, voltage);

            // --- Telemetry ---
            Pose p = drivetrain.follower.getPose();
            telemetry.addLine(autoPositioner.telemetry());
            telemetry.addData("Robot", "(%.1f, %.1f) @ %.0f°",
                    p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.addData("Ready to shoot", autoPositioner.isReadyToShoot());
            telemetry.addData("Avoiding", autoPositioner.isAvoiding());
            telemetry.addData("Goal fix", goalLocator.hasLiveFix()
                    ? String.format("LIVE (%d sightings) (%.1f,%.1f)",
                        goalLocator.getSightings(), goalLocator.getGoal().getX(), goalLocator.getGoal().getY())
                    : "fallback (tag not seen yet)");
            List<Obstacle> obs = mockObstacles.getObstacles();
            telemetry.addData("Obstacles", obs.size());
            for (int i = 0; i < obs.size(); i++) {
                Obstacle o = obs.get(i);
                telemetry.addData("  obs" + i, "(%.0f, %.0f) v=%.0f in/s", o.x, o.y, o.speed());
            }
            telemetry.addLine("A=engage  B=disengage  X=toggle mock obstacles");
            telemetry.update();
        }

        autoPositioner.disengage();
    }

    private void toggleMock() {
        mockEnabled = !mockEnabled;
        mockObstacles.setEnabled(mockEnabled);
    }
}
