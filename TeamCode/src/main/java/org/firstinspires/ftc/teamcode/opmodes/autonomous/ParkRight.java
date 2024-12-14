package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "ParkRight", group = "autonomous")
public class ParkRight extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        robot.drive_train.odo_move(-30,0,0,0.75);

    }

    @Override
    public void on_stop() {
    }
}