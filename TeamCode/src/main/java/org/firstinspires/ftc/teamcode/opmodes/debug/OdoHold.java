package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.LiveRobot;

@Autonomous(name="Odo Hold", group="autonomous")
public class OdoHold extends LinearOpMode {

    LiveRobot robot;

    int pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LiveRobot(this);
        robot.startup();

        waitForStart();

        // Hold position at 0, 0 forever
        robot.drive_train.odo_move(0.01, 0, 0, 1, -1, -1);

        robot.shutdown();
    }
}
