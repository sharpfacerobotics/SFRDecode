package org.firstinspires.ftc.teamcode;

import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


    @TeleOp(name = "EHS Testing", group = "Arm")
//    @Config // Enables configuration via FTC Dashboard
    public class Testing extends LinearOpMode {

        DcMotor backRightMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor frontLeftMotor;
        DcMotor fireWheel


        public void runOpMode() throws InterruptedException {
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            fireWheel = hardwareMap.get(DcMotor.class, "fireWheel");
            double ly = -gamepad1.left_stick_y; // y stick is reversed
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            waitForStart();

            while (opModeIsActive()) {
                frontLeftMotor.setPower(ly + lx + rx);
                backLeftMotor.setPower(ly - lx + rx);
                frontRightMotor.setPower(ly - lx - rx);
                backRightMotor.setPower(ly + lx - rx);
                if (gamepad1.right_trigger > 0.1) {
                    fireWheel.setPower(1);
                }
                }

            }
        }
    }
