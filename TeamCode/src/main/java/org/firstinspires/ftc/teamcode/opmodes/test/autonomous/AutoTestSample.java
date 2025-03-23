package org.firstinspires.ftc.teamcode.opmodes.test.autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.robots.TestRobot;

public class AutoTestSample {
    //// Variables ////
    private TestRobot robot;

    private static double fast;
    private static double slow;
    private static int sampleCount;

    public AutoTestSample(TestRobot robot, double fast, double slow) {
        this.robot = robot;
        this.fast = fast;
        this.slow = slow;
        sampleCount = 0;
    }

    public void sampleInit() {
        robot.drive_train.odo_reset(AutoConst.leftInitPose);

        robot.intake.auto_run = false;
        robot.arm.transfer_position();
        robot.arm.close_claw();
    }

    public void sampleStop() {

    }


    public void highBasket() {
        int sample = sampleCount;

        robot.drive_train.odo_drive(AutoConst.highBasketPoseOffset, fast); // quickly drive to pose

        if (sampleCount != 0) {
            while (!robot.reach.cur_limit_switch || !robot.lift.cur_limit_switch) { // Attempt to retract all slides
                robot.arm.waiting_position();
                robot.lift.min_lift();
                robot.reach.min_reach();
            }

            halt(1);

            boolean haveSample = (robot.intake.current_color != IntakeConst.SAMPLE_NONE);

            robot.arm.transfer_position(); // grab sample and raise lift to high basket

            halt(0.2);

            while (haveSample) {
                robot.arm.close_claw();

                halt(0.1);

                robot.lift.elevate_to(LiftConst.HIGH_BASKET);

                halt(0.3);

                if (robot.intake.current_color != IntakeConst.SAMPLE_NONE) {
                    robot.arm.open_claw();
                    robot.lift.min_lift();
                    halt(0.5);
                } else {
                    haveSample = false;
                    sampleCount++;
                }
            }

            robot.intake.auto_run = false;

            halt(1);

            if (sample != sampleCount) {
                robot.drive_train.odo_drive(AutoConst.highBasketPose, slow); // move slower to pose
                robot.arm.basket_position();

                halt(1);

                robot.arm.open_claw(); //deposit sample in basket
            }
        }
    }

    public void highBasketFirst() {
        sampleCount++;

        robot.drive_train.odo_drive(AutoConst.highBasketPoseOffset, fast); // quickly drive to pose

        robot.arm.transfer_position(); // grab sample and raise lift to high basket

        halt(0.2);

        robot.arm.close_claw();

        halt(0.1);

        robot.lift.elevate_to(LiftConst.HIGH_BASKET);

        halt(1.5);

        robot.drive_train.odo_drive(AutoConst.highBasketPose, slow); // move slower to pose
        robot.arm.basket_position();

        halt(1);

        robot.arm.open_claw(); // deposit sample in basket
    }

    public void sampleReachIntake(int position) {
        robot.drive_train.odo_drive(samplePose(position, true), fast); // quickly driving to pose

        halt(0.4);

        robot.drive_train.odo_drive(samplePose(position, false), slow); // move slower to pose

        robot.lift.elevate_to(LiftConst.INIT);
        robot.arm.waiting_position();
        robot.arm.open_claw();

        robot.intake.intake_pitch(IntakeConst.INTAKE);
        robot.intake.auto_run = true;

        while (robot.drive_train.moving) {
            halt(0.1);
        }

        robot.reach.extend_to(400);

        boolean wiggle = true;
        double start_intake = robot.opmode.getRuntime();
        while (robot.intake.current_color == IntakeConst.SAMPLE_NONE && (robot.opmode.getRuntime() - start_intake < 2.0)) {
            if (robot.cycle % 20 == 0) {
                robot.drive_train.odo_wiggle(wiggle ? 0.2 : -0.2);
                wiggle = !wiggle;
            }
        }

        robot.intake.intake_pitch(IntakeConst.TRANS);

        halt(0.1);

        robot.reach.min_reach();
    }

    public void subPark() {
        Pose start = robot.drive_train.lcs.get_pose();
        robot.drive_train.odo_drive(start.x, AutoConst.subPark.y, AutoConst.subPark.a, fast);

        halt(0.5);

        robot.lift.min_lift();

        halt(1.5);

        robot.drive_train.odo_drive(AutoConst.subPark, 0.3);

        robot.arm.park_position();
        robot.arm.close_claw();

        halt(3);
    }

    public Pose samplePose(int position, boolean offset) {
        if (position == AutoConst.SAMPLE_RIGHT) {
            return offset ? AutoConst.rightSamplePoseOffset : AutoConst.rightSamplePose;
        }
        if (position == AutoConst.SAMPLE_MID) {
            return offset ? AutoConst.midSamplePoseOffset : AutoConst.midSamplePose;
        }
        if (position == AutoConst.SAMPLE_LEFT) {
            return offset ? AutoConst.leftSamplePoseOffset : AutoConst.leftSamplePose;
        }
        if (position == AutoConst.SAMPLE_PARTNER) {
            return offset ? AutoConst.partnerSamplePoseOffset : AutoConst.partnerSamplePose;
        }
        return null;
    }

    public void halt(double seconds) {
        double start = robot.opmode.getRuntime();
        while ((robot.opmode.getRuntime() - start) < seconds && robot.opmode.opModeIsActive());
    }
}
