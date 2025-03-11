package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.robots.LiveRobot;

public class AutoSpecimen {
    //// Variables ////
    private LiveRobot robot;

    public static double fast;
    public static double slow;
    private static int sampleCount;

    public AutoSpecimen(LiveRobot robot, double fast, double slow) {
        this.robot = robot;
        this.fast = fast;
        this.slow = slow;
        sampleCount = 0;
    }

    public void specimenInit() {
        robot.drive_train.odo_reset(AutoConst.rightInitPose);
        robot.drive_train.auto = true;

        robot.intake.auto_run = false;
        robot.arm.waiting_position();
        robot.arm.open_claw();
    }

    public void specimenStart() {
    }

    public void specimenStop() {
        robot.intake.auto_run = false;
        robot.lift.shutdown();
        robot.drive_train.shutdown();

    }

    public void halt(double seconds) {
        robot.opmode.resetRuntime();
        while (robot.opmode.getRuntime() < seconds && robot.opmode.opModeIsActive());
    }
}
