package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "twoSampleAuto", group = "autonomous")
public class twoSampleAuto extends LiveAutoBase {

    double fast = 0.66;
    double slow = 0.25;

    int sampleCount = 0;
    @Override
    public void on_init() {
        robot.drive_train.odo_reset(AutoConst.leftInitPose);
        robot.drive_train.auto = true;
        robot.intake.auto_run = false;
    }

    @Override
    public void on_start() {

        highBasket();
        sampleIntake(AutoConst.SAMPLE_RIGHT);
        highBasket();
        subPark();

    }

    @Override
    public void on_stop() {

    }

    // working
    public void highBasket() {
        int sample = sampleCount;

        robot.drive_train.odo_drive(AutoConst.highBasketPoseOffset, fast); // quickly drive to pose

        while (!(robot.reach.cur_limit_switch && robot.lift.cur_limit_switch)) { // Attempt to retract all slides
            robot.arm.waiting_position();
            robot.lift.min_lift();
            robot.reach.min_reach();
            halt(0.1);
        }

        boolean haveSample = (robot.intake.current_color != IntakeConst.SAMPLE_NONE);

        robot.arm.transfer_position(); // grab sample and raise lift to high basket

        halt(0.2);

        robot.intake.auto_run = false;

        while (haveSample) {
            robot.arm.close_claw();

            halt(0.1);

            robot.lift.elevate_to(LiftConst.HIGH_BASKET);

            halt(0.2);

            if (robot.intake.current_color != IntakeConst.SAMPLE_NONE) {
                robot.arm.open_claw();
                robot.lift.min_lift();
                halt(1);
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
        robot.drive_train.odo_drive(samplePose(position, true), fast); // quickly driving to pose

        halt(0.2);

        robot.lift.elevate_to(LiftConst.INIT);
        robot.arm.waiting_position();

        halt(0.2);

        robot.drive_train.odo_drive(samplePose(position, false), slow); // move slower to pose
        robot.intake.intake_pitch(IntakeConst.INTAKE);
        robot.intake.auto_run = true;

        while (robot.drive_train.moving) {
            halt(0.1);
        }

        boolean wiggle = true;
        double cur_a = robot.drive_train.target_a;
        while (robot.intake.current_color == IntakeConst.SAMPLE_NONE && robot.drive_train.lcs.y < 60) {
            if (robot.cycle % 20 == 0) {
                robot.drive_train.odo_creep_wiggle(cur_a, 1, -1, (wiggle ? 0.1 : -0.1), slow);
                wiggle = !wiggle;
            }
        }

        robot.intake.intake_pitch(IntakeConst.TRANS);
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
        Pose start = robot.drive_train.lcs.get_pose();
        robot.drive_train.odo_drive(start.x, AutoConst.subPark.y, AutoConst.subPark.a, fast);

        halt(0.5);

        robot.lift.min_lift();

        halt(1.5);

        robot.drive_train.odo_drive(AutoConst.subPark, slow);

        robot.arm.park_position();
        robot.arm.close_claw();

        halt(4);
    }

    public void humanPark() {
        robot.lift.min_lift();
        robot.drive_train.odo_drive(AutoConst.humanPark, fast);
    }
}