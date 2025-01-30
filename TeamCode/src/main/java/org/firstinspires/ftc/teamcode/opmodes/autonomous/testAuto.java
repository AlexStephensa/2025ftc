package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testAuto", group = "autonomous")
public class testAuto extends LiveAutoBase {

    double drive_speed = 0.5;

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        robot.drive_train.setCurrentPose(8.5, 32.5, Math.PI/2);
        Pose highBasket = new Pose(14, 14, Math.PI/4);
        robot.drive_train.drive_to_pose(highBasket, drive_speed, 1);
        halt(10);

    }

    @Override
    public void on_stop() {
    }
}