package org.firstinspires.ftc.teamcode.opmodes.test.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.opmodes.test.TestAutoBase;

@Autonomous(name = "TEST 0+4+sub", group = "autonomous")
@Disabled
public class testFourSampleAuto extends TestAutoBase {

    AutoTestSample auto;

    @Override
    public void on_init() {
        RobotLog.dd("---", "auto-on_init");

        auto = new AutoTestSample(robot, 0, 0);
        auto.sampleInit();
    }

    @Override
    public void on_start() {
        RobotLog.dd("---", "auto-on_start");

        auto.highBasketFirst();
        auto.sampleReachIntake(AutoConst.SAMPLE_RIGHT);
        auto.highBasket();
        auto.sampleReachIntake(AutoConst.SAMPLE_MID);
        auto.highBasket();
        auto.sampleReachIntake(AutoConst.SAMPLE_LEFT);
        auto.highBasket();
        auto.subPark();
    }

    @Override
    public void on_stop() {
        RobotLog.dd("---", "auto-on_stop");

        auto.sampleStop();
    }
}