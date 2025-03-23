package org.firstinspires.ftc.teamcode.opmodes.live.autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;
import org.firstinspires.ftc.teamcode.opmodes.live.autonomous.AutoSample;

@Autonomous(name = "0+3+sub", group = "autonomous", preselectTeleOp = "Teleop Live")
public class threeSampleAuto extends LiveAutoBase {

    AutoSample auto;
    @Override
    public void on_init() {
        auto = new AutoSample(robot, 0.66, 0.5);
        auto.sampleInit();
    }

    @Override
    public void on_start() {
        auto.highBasketFirst();
        auto.sampleReachIntake(AutoConst.SAMPLE_RIGHT);
        auto.highBasket();
        auto.sampleReachIntake(AutoConst.SAMPLE_MID);
        auto.highBasket();
        auto.subPark();
    }

    @Override
    public void on_stop() {
        auto.sampleStop();
    }
}