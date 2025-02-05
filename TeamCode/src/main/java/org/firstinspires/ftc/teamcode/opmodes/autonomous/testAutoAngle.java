package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testAuto", group = "autonomous")
public class testAutoAngle extends LiveAutoBase {

    double drive_speed = 0.5;

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        Pose angleTest = new Pose(0, 0, 0);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a + Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a + Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a + Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a + Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a - Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a - Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a - Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);
        halt(2);
        angleTest.set_a(angleTest.a - Math.PI / 4);
        robot.drive_train.odo_move(angleTest, 0.5, 2);

    }

    @Override
    public void on_stop() {
    }

}