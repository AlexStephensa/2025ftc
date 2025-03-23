package org.firstinspires.ftc.teamcode.opmodes.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.opmodes.test.TestAutoBase;

@Autonomous(name = "testHumanPark", group = "autonomous")
@Disabled
public class testHumanPark extends TestAutoBase {

    @Override
    public void on_init() {
        RobotLog.dd("---", "auto-on_init");
        robot.drive_train.odo_reset(AutoConst.rightInitPose);
    }

    @Override
    public void on_start() {
        RobotLog.dd("---", "auto-on_start");
        robot.drive_train.odo_move(AutoConst.humanParkRight, 0.5);

        halt(5);
    }

    @Override
    public void on_stop() {
        RobotLog.dd("---", "auto-on_stop");
    }
}