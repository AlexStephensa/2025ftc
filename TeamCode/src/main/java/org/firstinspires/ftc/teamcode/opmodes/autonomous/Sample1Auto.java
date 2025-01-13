package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "1 Sample Auto", group = "autonomous")
public class Sample1Auto extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {

        /*final Path samplePath = new Path();
        samplePath.speed(0.5);
        samplePath
                .addPoint(new PathPoint(0, 0))
                .timeout(5)
                .addPoint(new PathPoint(2, 0));
        robot.drive_train.follow_curve_path(samplePath);*/

        robot.drive_train.odo_reset(0.0, 0.0, 0.0);
        robot.drive_train.odo_move(2.0, 0.0, 0.0, 0.5);
        robot.wait(500);
        robot.drive_train.odo_move(2.0, 0.0, 0.0, 0.5);

    }

    @Override
    public void on_stop() {
    }
}