package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testComponents", group = "autonomous")
public class testComponents extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        robot.intake.setAuto_run(false);

        halt(0.1);

        robot.intake.setAuto_run(true);

        halt(5);

        robot.intake.setAuto_run(false);

        halt(5);
    }

    @Override
    public void on_stop() {
    }

}