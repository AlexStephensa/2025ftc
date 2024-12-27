package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.live.DriveTrain;
import org.firstinspires.ftc.teamcode.components.live.Intake;
import org.firstinspires.ftc.teamcode.components.live.Lift;
import org.firstinspires.ftc.teamcode.components.live.Reach;
import org.firstinspires.ftc.teamcode.coyote.geometry.Point;

import java.util.ArrayList;

public class LiveRobot extends Robot {
    public DriveTrain       drive_train;
    public Lift             lift;
    public Reach            reach;
    public Intake           intake;

    FtcDashboard            dashboard;

    ArrayList<Point> robot_movement = new ArrayList<Point>();

    {
        name = "Boris";
    }

    public LiveRobot(LinearOpMode opmode) {
        super(opmode);

        drive_train     = new DriveTrain(this);
        reach           = new Reach(this);
        lift            = new Lift(this);
        intake          = new Intake(this);

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void updateTelemetry() {
        super.updateTelemetry();
    }
}