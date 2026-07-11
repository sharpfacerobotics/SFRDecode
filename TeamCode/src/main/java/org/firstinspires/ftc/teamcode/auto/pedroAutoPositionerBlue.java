package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.nav.FieldConfig;

@Configurable
@Autonomous(name="PedroAutoPositionerBlue", group="Autonomous")
public class pedroAutoPositionerBlue extends PositionerAuto {

    @Override
    protected void definePoses() {
        startPose = new Pose(87.47252747252746, 8, Math.toRadians(90)).mirror();
        shootPose = startPose;   // unused — AutoPositioner picks the scoring pose itself
    }

    @Override
    protected Pose goalPose() {
        return FieldConfig.BLUE_GOAL;
    }

    @Override
    protected Pose intakePose() {
        return new Pose(142.57660626029653, 9.043956043956046, Math.toRadians(0)).mirror();
    }
}
