package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "test", group = "autonomous")
public class testAutoAngle extends LiveAutoBase {

    double drive_speed = 0.5;

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        halt(35);

    }

    @Override
    public void on_stop() {
    }

}