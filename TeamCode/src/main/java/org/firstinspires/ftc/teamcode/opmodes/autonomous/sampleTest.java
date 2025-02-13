package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "sampleTest", group = "autonomous")
public class sampleTest extends LiveAutoBase {

    double fast = 0.66;
    double slow = 0.25;

    int sampleCount = 0;
    @Override
    public void on_init() {
        robot.drive_train.odo_reset(AutoConst.leftInitPose);
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

        robot.drive_train.odo_drive(AutoConst.highBasketPoseOffset, fast); // quickly drive to pose

        while (!(robot.reach.cur_limit_switch && robot.lift.cur_limit_switch)) { // Attempt to retract all slides
            robot.arm.waiting_position();
            robot.lift.min_lift();
            robot.reach.min_reach();
        }

        halt(0.5);

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

        halt(1);

        if (sample != sampleCount) {
            robot.drive_train.odo_drive(AutoConst.highBasketPose, 0.5); // move slower to pose
            robot.arm.basket_position();

            halt(1);

            robot.arm.open_claw(); // deposit sample in basket
        }
    }

    public void sampleIntake(int position) {
        robot.drive_train.odo_drive(samplePose(position, true), 1); // quickly driving to pose

        halt(0.2);

        robot.lift.elevate_to(LiftConst.INIT);
        robot.arm.waiting_position();

        halt(0.2);

        robot.drive_train.odo_drive(samplePose(position, false), 0.5); // move slower to pose
        robot.intake.intake_intake();

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
        robot.drive_train.odo_drive(start.x, AutoConst.subPark.y, AutoConst.subPark.a, fast);
        robot.lift.min_lift();

        halt(2);

        robot.drive_train.odo_drive(AutoConst.subPark, slow);
        robot.arm.basket_position();

        halt(2);
    }
}