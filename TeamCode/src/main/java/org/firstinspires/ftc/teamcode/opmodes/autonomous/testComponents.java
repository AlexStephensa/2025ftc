/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testComponents", group = "autonomous")
public class testComponents extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        robot.intake.intake_pitch(IntakeConst.INTAKE);

        halt(0.1);

        robot.intake.auto_run = true;

        halt(5);

        robot.intake.auto_run = false;

        halt(5);
    }

    @Override
    public void on_stop() {
    }

}*/