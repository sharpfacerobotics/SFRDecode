package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name="PedroAutoCloseRed", group="Autonomous")
public class pedroAutoCloseRed extends CloseAuto {
    @Override
    protected void definePoses() {
        startPose   = new Pose(20.77, 122.99, Math.toRadians(145)).mirror();
        shootPose   = new Pose(58.78, 84.27,  Math.toRadians(140)).mirror();
        intakeOne   = new Pose(21,    84.43,  Math.toRadians(185)).mirror();
        intakeTwo   = new Pose(16.8,  59,     Math.toRadians(185)).mirror();
        intakeThree = new Pose(16.3,  35,     Math.toRadians(185)).mirror();

        controlPointToIntakeTwo   = new Pose(55.51, 54.81, Math.toRadians(185)).mirror();
        controlPointToIntakeThree = new Pose(90, 31.2, Math.toRadians(185)).mirror();
    }
}