package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "simpleHumanPark", group = "autonomous", preselectTeleOp = "Teleop Live")
public class simpleHumanPark extends LiveAutoBase {

    AutoSample auto;
    @Override
    public void on_init() {
        robot.drive_train.odo_reset(AutoConst.rightInitPose);
    }

    @Override
    public void on_start() {
        robot.drive_train.odo_move(AutoConst.humanParkRight, 0.5);

        halt(5);
    }

    @Override
    public void on_stop() {

    }
}