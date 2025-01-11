package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "1 Sample Auto", group = "autonomous")
public class Sample1Auto extends LiveAutoBase {

    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        final Path samplePath = new Path();
        samplePath
                .addPoint(new PathPoint(20, 20))
                /*.addPoint(new PathPoint(5, 0).addAction(() -> {
                    robot.intake.spin(1);
                    robot.intake.cradle_intake();
                }))
                .addPoint(new PathPoint(-4, -12*2).addAction(() -> {
                    intake_path.speed(0.5);
                }))
                .addPoint(new PathPoint(0, -12*4))
                .positionPrecision(5)
                .headingPrecision(0.04)
                .constantHeading(-Math.PI/2)
                .followRadius(18)
                .speed(1)
                .until(() -> robot.intake.has_element())*/;
                robot.drive_train.follow_curve_path(samplePath);

    }

    @Override
    public void on_stop() {
    }
}