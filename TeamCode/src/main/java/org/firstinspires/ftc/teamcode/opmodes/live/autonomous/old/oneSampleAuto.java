package org.firstinspires.ftc.teamcode.opmodes.live.autonomous.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;
import org.firstinspires.ftc.teamcode.opmodes.live.autonomous.AutoSample;

@Autonomous(name = "0+1+sub", group = "autonomous", preselectTeleOp = "Teleop Live")
@Disabled
public class oneSampleAuto extends LiveAutoBase {

    AutoSample auto;
    @Override
    public void on_init() {
        auto = new AutoSample(robot, 0.66, 0.5);
        auto.sampleInit();
    }

    @Override
    public void on_start() {
        auto.highBasket();
        auto.subPark();
    }

    @Override
    public void on_stop() {
        auto.sampleStop();
    }
}