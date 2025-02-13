package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.live.Arm;
import org.firstinspires.ftc.teamcode.components.live.DriveTrain;
import org.firstinspires.ftc.teamcode.components.live.Intake;
import org.firstinspires.ftc.teamcode.components.live.Lift;
import org.firstinspires.ftc.teamcode.components.live.Reach;

public class LiveRobot extends Robot {
    public DriveTrain       drive_train;
    public Lift             lift;
    public Reach            reach;
    public Intake           intake;
    public Arm              arm;

    FtcDashboard            dashboard;

    {
        name = "BORIS";
    }

    public LiveRobot(LinearOpMode opmode) {
        super(opmode);

        drive_train     = new DriveTrain(this);
        reach           = new Reach(this);
        lift            = new Lift(this);
        intake          = new Intake(this);
        arm             = new Arm(this);

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