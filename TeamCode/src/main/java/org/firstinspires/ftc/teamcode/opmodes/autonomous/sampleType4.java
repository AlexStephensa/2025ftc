package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "4Sample+Park", group = "autonomous")
public class sampleType4 extends LiveAutoBase {

    public double robotSpeed = 0.5;
    public int sampleCount = 0;

    @Override
    public void on_init() {
        robot.drive_train.odo_reset(AutoConst.leftInitPose);

        robot.addWarning("ROBS NOSE IS TOO LARGE");
    }

    @Override
    public void on_start() {

        highBasket();
        sampleIntake(AutoConst.SAMPLE_RIGHT);
        highBasket();
        sampleIntake(AutoConst.SAMPLE_MID);
        highBasket();
        sampleIntake(AutoConst.SAMPLE_LEFT);
        highBasket();
        subPark();

    }

    @Override
    public void on_stop() {

    }

    public void highBasket() {
        int sample = sampleCount;
        robot.drive_train.odo_drive_towards(AutoConst.highBasketPoseOffset, 1); // quickly drive to pose

        while (!(robot.reach.cur_limit_switch && robot.lift.cur_limit_switch)) { // Attempt to retract all slides
            robot.arm.waiting_position();
            robot.lift.min_lift();
            robot.reach.min_reach();
        }

        boolean haveSample = (robot.intake.current_color != IntakeConst.SAMPLE_NONE);

        robot.arm.transfer_position(); // grab sample and raise lift to high basket
        halt(0.1);

        while (haveSample) {
            robot.arm.close_claw();
            halt(0.1);
            robot.lift.elevate_to(LiftConst.HIGH_BASKET);
            halt(0.2);
            if (robot.intake.current_color != IntakeConst.SAMPLE_NONE) {
                robot.arm.open_claw();
                robot.lift.min_lift();
                halt(0.5);
            } else {
                haveSample = false;
                sampleCount++;
            }
        }

        if (sample != sampleCount) {
            robot.arm.basket_position();

            robot.drive_train.odo_move(AutoConst.highBasketPose, 0.5, 1); // move slower to pose

            robot.arm.open_claw(); // deposit sample in basket
        }
    }

    public void sampleIntake(int position) {
        robot.drive_train.odo_drive_towards(samplePose(position, true), 1); // quickly driving to pose

        robot.lift.elevate_to(LiftConst.INIT);
        robot.arm.waiting_position();

        robot.intake.intake_intake();
        robot.reach.extend_to(200);

        robot.drive_train.odo_drive_towards(samplePose(position, false), 0.5); // move slower to pose

        while (robot.intake.current_color == IntakeConst.SAMPLE_NONE) {
            robot.intake.intake_run_auto(1);
        }

        robot.intake.intake_transfer();
        halt(0.1);
        robot.reach.min_reach();
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

    public void subPark() {
        Pose start = robot.drive_train.getCurrentPose();
        robot.drive_train.odo_drive_towards(start.x, AutoConst.subPark.y, AutoConst.subPark.a, 1);

        halt(1);

        robot.arm.basket_position(); // for the robot to touch the low rung
        robot.drive_train.odo_drive_towards(AutoConst.subPark, 1);
        halt(1);
    }
}