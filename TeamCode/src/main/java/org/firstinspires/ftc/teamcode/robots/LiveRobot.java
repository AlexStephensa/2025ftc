package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.components.live.Arm;
import org.firstinspires.ftc.teamcode.components.live.DriveTrain;
import org.firstinspires.ftc.teamcode.components.live.Intake;
import org.firstinspires.ftc.teamcode.components.live.LEDControl;
import org.firstinspires.ftc.teamcode.components.live.Lift;
import org.firstinspires.ftc.teamcode.components.live.Reach;

public class LiveRobot extends Robot {
    public DriveTrain       drive_train;
    public Lift             lift;
    public Reach            reach;
    public Intake           intake;
    public Arm              arm;

    public LEDControl led_control;

    FtcDashboard            dashboard;

    {
        name = "BORIS";
    }

    public LiveRobot(LinearOpMode opmode) {
        super(opmode);

        RobotLog.dd("---", "drive_train");
        drive_train     = new DriveTrain(this);
        RobotLog.dd("---", "lift");
        lift            = new Lift(this);
        RobotLog.dd("---", "reach");
        reach           = new Reach(this);
        RobotLog.dd("---", "intake");
        intake          = new Intake(this);
        RobotLog.dd("---", "arm");
        arm             = new Arm(this);

        RobotLog.dd("---", "led_control");
        led_control     = new LEDControl(this);
        RobotLog.dd("---", "dashboard");
        dashboard       = FtcDashboard.getInstance();
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