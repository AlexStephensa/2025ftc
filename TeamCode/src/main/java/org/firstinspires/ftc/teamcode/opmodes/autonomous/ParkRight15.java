package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "ParkRight15", group = "autonomous")
public class ParkRight15 extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        robot.drive_train.odo_move(-5,0,0,0.75);
        halt(5);
        robot.drive_train.odo_move(-5,0,0,0.75);
        halt(5);
        robot.drive_train.odo_move(-5,0,0,0.75);
        halt(5);
        robot.drive_train.odo_move(10,0,0,0.75);
        halt(5);

    }

    @Override
    public void on_stop() {
    }
}