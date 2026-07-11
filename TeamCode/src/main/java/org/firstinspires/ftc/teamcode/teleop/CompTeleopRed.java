/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.Launcher.TRIGGER_START_POS;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.nav.AutoPositioner;
import org.firstinspires.ftc.teamcode.robot.nav.FieldConfig;
import org.firstinspires.ftc.teamcode.robot.nav.GoalLocator;
import org.firstinspires.ftc.teamcode.robot.nav.LimelightObstacleSource;
import org.firstinspires.ftc.teamcode.robot.nav.MockObstacleSource;
import org.firstinspires.ftc.teamcode.robot.nav.ObstacleSource;

@TeleOp(name="CompTeleopRed", group="Linear OpMode")
@Configurable
public class CompTeleopRed extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();
    private Intake intake = null;
    private Launcher launcher = null;
    private Drivetrain drivetrain = null;
    private Vision vision = null;

    // Auto-positioner: live-replanned spline to the best scoring pose, with obstacle dodging.
    // Obstacle feed defaults to none (no sensing hardware yet); flip the flags to demo/enable.
    public static boolean USE_MOCK_OBSTACLES      = false;
    public static boolean USE_LIMELIGHT_OBSTACLES = false;
    private AutoPositioner autoPositioner = null;
    private GoalLocator goalLocator = null;

    public static double PUSH_START_POS = 0;


    private enum AutoDriveState {
        MANUAL,
        DRIVING_TO_SHOOT,
        LOCKING_ON,
        SHOOTING,
        DRIVING_TO_INTAKE,
        INTAKING,
    }

    private AutoDriveState autoDriveState = AutoDriveState.MANUAL;
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    private void setAutoDriveState(AutoDriveState newState) {
        autoDriveState = newState;
        autoDriveTimer.reset();
    }

    /** Start a positioner-driven shoot leg (sets autoDriving so joystick-cancel still works). */
    private void engagePositioner() {
        autoPositioner.engage();
        drivetrain.autoDriving = true;
    }
    private double autoDriveTime = 5.5;
    private double lockOnTime = 2.0;
    private double shootTime = 6.5;
    private double intakeTime = 2.0;
    private boolean loopWantedClose = false;
    private boolean loopWantedFar = false;


    @Override
    public void runOpMode() {
        //configurePinpoint();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robot.init(hardwareMap);

        intake = new Intake(robot);

        launcher = new Launcher(robot);
        launcher.init();

        drivetrain = new Drivetrain(robot);
        drivetrain.init(hardwareMap);

        vision = new Vision(robot);
        vision.init();

        ObstacleSource obstacleSource;
        if (USE_LIMELIGHT_OBSTACLES) {
            obstacleSource = new LimelightObstacleSource(robot.limelight,
                    () -> drivetrain.follower.getPose());
        } else {
            MockObstacleSource mock = new MockObstacleSource();
            mock.setEnabled(USE_MOCK_OBSTACLES);
            obstacleSource = mock;
        }
        goalLocator = new GoalLocator(FieldConfig.RED_GOAL);
        autoPositioner = new AutoPositioner(drivetrain.follower, obstacleSource, FieldConfig.RED_GOAL);
        autoPositioner.setGoalLocator(goalLocator);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        // Set initial positions
        robot.Trigger.setPosition(TRIGGER_START_POS);
        robot.push.setPosition(PUSH_START_POS);

        telemetry.addData("Status", "Waiting for limelight...");
        telemetry.update();

        // Tell Pedro to start exactly where Auto finished
        drivetrain.follower.setStartingPose(PoseStorage.currentPose);


        while (!isStarted()) {
            LLResult result = vision.update();
            if (vision.setPose()) {
                drivetrain.follower.setPose(vision.startPose);
                telemetry.addData("Status", "Localized!");
                telemetry.addData("Pose", "%.1f, %.1f", vision.startPose.getX(), vision.startPose.getY());
            } else {
                telemetry.addData("Status", "No tag visible - aim at AprilTag");
            }
            telemetry.update();
            if (result != null && result.isValid()) {
                telemetry.addData("PedroX from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[0]);
                telemetry.addData("PedroY from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[1]);
                telemetry.addData("Pedro Heading from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[2]);
            }
        }

        waitForStart();
        runtime.reset();
        robot.imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drivetrain.follower.update();
            LLResult result = vision.update();
            double distance = vision.getDistance();
            double voltage = robot.myControlHubVoltageSensor.getVoltage();

            // Live goal calibration: refine the goal position whenever its tag is in view.
            if (result != null && result.isValid()) {
                goalLocator.update(drivetrain.follower.getPose(), result.getTx(), distance);
            }

            if (result == null || !result.isValid()) {
                telemetry.addLine("Not Locked On");
            } else {
                telemetry.addLine("Locked On");
            }

            // Start auto drive on button press. The shoot leg is driven by the AutoPositioner
            // (live-chosen scoring pose, spline replanning, obstacle dodging); A/B still select
            // which INTAKE pose the loop uses afterwards.
            if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && !gamepad1.a && !gamepad1.b) {
                engagePositioner();
                setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
            } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && gamepad1.a && !gamepad1.b) {
                engagePositioner();
                setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
            } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && !gamepad1.a && gamepad1.b) {
                engagePositioner();
                setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
                loopWantedClose = true;
            } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && gamepad1.a && gamepad1.b) {
                engagePositioner();
                setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
                loopWantedFar = true;
            }

            // Cancel auto drive if driver touches the joystick
            if (drivetrain.autoDriving && (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                    Math.abs(gamepad1.left_stick_x) > 0.1 ||
                    Math.abs(gamepad1.right_stick_x) > 0.1)) {
                autoPositioner.disengage();
                drivetrain.cancelAutoDrive();
                setAutoDriveState(AutoDriveState.MANUAL);
                loopWantedClose = false;
                loopWantedFar = false;
            }

            double yaw = 0;

            switch (autoDriveState) {
                case MANUAL:
                    loopWantedClose = false;
                    loopWantedFar = false;
                    double axial   = -gamepad1.left_stick_y;
                    double lateral =  gamepad1.left_stick_x;
                    yaw = vision.getYaw(gamepad1.right_stick_x, gamepad1.left_bumper, result);
                    drivetrain.drive(axial, lateral, yaw);
                    break;

                case DRIVING_TO_SHOOT:
                    autoPositioner.update();
                    telemetry.addLine(autoPositioner.telemetry());
                    if (autoPositioner.isReadyToShoot()) {
                        telemetry.addLine("Arrived");
                        autoPositioner.disengage();   // freeze; LOCKING_ON takes over aiming
                        setAutoDriveState(AutoDriveState.LOCKING_ON);
                    }
                    if (autoDriveTimer.seconds() > autoDriveTime) {
                        telemetry.addLine("Auto Drive Timeout");
                        autoPositioner.disengage();
                        setAutoDriveState(AutoDriveState.MANUAL);
                        drivetrain.cancelAutoDrive();
                    }
                    break;

                case LOCKING_ON:
                    if (!vision.isLockedOn()) {
                        yaw = vision.getYaw(0.0, true, result);
                        drivetrain.drive(0, 0, yaw);
                        telemetry.addLine("Aligning");
                    } else {
                        telemetry.addLine("Locked On");
                        if (result != null && result.isValid()) {
                            drivetrain.follower.holdPoint(vision.limelightToPedroPose(result.getBotpose_MT2()));
                        }
                        launcher.tripleShotStarted = false;
                        setAutoDriveState(AutoDriveState.SHOOTING);
                    }
                    if (autoDriveTimer.seconds() > lockOnTime) {
                        telemetry.addLine("Auto Lock Timeout");
                        setAutoDriveState(AutoDriveState.MANUAL);
                        drivetrain.cancelAutoDrive();
                    }
                    break;

                case SHOOTING:
                    if (result != null && result.isValid()) {
                        drivetrain.follower.holdPoint(vision.limelightToPedroPose(result.getBotpose_MT2()));
                    }
                    launcher.update(0.0, true, vision.hasTarget(), distance, voltage);
                    telemetry.addLine("Shooting");
                    if (launcher.tripleShotStarted && !launcher.isTripleShotActive()) {
                        telemetry.addLine("Done");
                        if (!loopWantedClose && !loopWantedFar) {
                            drivetrain.cancelAutoDrive();
                            setAutoDriveState(AutoDriveState.MANUAL);
                        } else if (loopWantedClose){
                            drivetrain.startAutoDrive(drivetrain.CLOSE_RED_INTAKE_POSE, drivetrain.CLOSE_RED_INTAKE_CONTROL_POSE);
                            setAutoDriveState(AutoDriveState.DRIVING_TO_INTAKE);
                        } else if (loopWantedFar) {
                            drivetrain.startAutoDrive(drivetrain.FAR_RED_INTAKE_POSE);
                            setAutoDriveState(AutoDriveState.DRIVING_TO_INTAKE);
                        }
                    }
                    if (autoDriveTimer.seconds() > shootTime) {
                        telemetry.addLine("TripleShot Timeout");
                        setAutoDriveState(AutoDriveState.MANUAL);
                        drivetrain.cancelAutoDrive();
                    }
                    break;

                case DRIVING_TO_INTAKE:
                    telemetry.addLine("Following Path");
                    if (!drivetrain.follower.isBusy()) {
                        telemetry.addLine("Arrived");
                        setAutoDriveState(AutoDriveState.INTAKING);
                    }
                    if (autoDriveTimer.seconds() > autoDriveTime) {
                        telemetry.addLine("Auto Drive Timeout");
                        setAutoDriveState(AutoDriveState.MANUAL);
                        drivetrain.cancelAutoDrive();
                    }
                    break;

                case INTAKING:
                    intake.update(true, false, false, false, false);
                    intake.setTransfer(false, true);
                    if (autoDriveTimer.seconds() > intakeTime) {
                        engagePositioner();
                        setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
                    }
                    break;

                default:
                    telemetry.addLine("Unknown State/ No State Set");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    drivetrain.cancelAutoDrive();
                    break;
            }

            if (!drivetrain.autoDriving) {
                intake.update(gamepad2.a, gamepad2.b, gamepad1.y, gamepad2.x, launcher.isTripleShotActive());
                intake.setTransfer(gamepad2.right_bumper, gamepad2.left_bumper);
                launcher.update(gamepad2.right_trigger, gamepad2.y, vision.hasTarget(), distance, voltage);
            }

            if (result != null && result.isValid()) {
                telemetry.addData("Location: ", vision.limelightToPedroPose(result.getBotpose_MT2(), true));
            }
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
            telemetry.addData("Launch State", launcher.getLaunchState());
            telemetry.addData("Current Launch Power", "%.2f", launcher.getCurrentLaunchPower());
            telemetry.addData("distance: ", distance);
            telemetry.update();
        }
    }
}
