/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "0+2+sub", group = "autonomous", preselectTeleOp = "Teleop Live")
public class twoSampleAuto extends LiveAutoBase {

    AutoSample auto;
    @Override
    public void on_init() {
        auto = new AutoSample(robot, 0.66, 0.5);
        auto.sampleInit();
    }

    @Override
    public void on_start() {
        auto.highBasket();
        auto.sampleIntake(AutoConst.SAMPLE_RIGHT);
        auto.highBasket();
        auto.subPark();
    }

    @Override
    public void on_stop() {
        auto.sampleStop();
    }
}*/