package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;
import org.firstinspires.ftc.teamcode.robots.LiveRobot;

@Autonomous(name = "ParkLeft15", group = "autonomous")
public class ParkLeft15 extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        robot.drive_train.odo_move(15,0,0,0.75);

    }

    @Override
    public void on_stop() {
    }
}