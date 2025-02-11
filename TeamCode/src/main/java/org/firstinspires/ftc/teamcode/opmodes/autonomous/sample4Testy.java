package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "sample4Testy", group = "autonomous")
public class sample4Testy extends LiveAutoBase {
    public int nextSample; // what part of the sample cycle the robot is on

    @Override
    public void on_init() {
        robot.arm.transfer_position();
        robot.arm.close_claw();
        robot.drive_train.odo_reset(AutoConst.leftInitPose);
        nextSample = AutoConst.SAMPLE_RIGHT; // where in the intake loop to start
    }

    @Override
    public void on_start() {

        highBasket();
        sampleIntake();
        highBasket();
        sampleIntake();
        highBasket();
        sampleIntake();
        highBasket();
        subPark();

    }

    @Override
    public void on_stop() {

    }

    public void highBasket() {
        robot.drive_train.odo_drive_towards(AutoConst.highBasketPoseOffset, 1); // quickly driving to pose

        /*if (!robot.reach.limit_switchR.getState() && !robot.intake.current_color_name.equals("NONE")) {
            robot.arm.transfer_position();
            robot.arm.close_claw();
        }*/

        //robot.lift.elevate_to(LiftConst.HIGH_BASKET);
        //robot.arm.basket_position(); // moving lift and arm to deposit positions

        robot.drive_train.odo_move(AutoConst.highBasketPose, 0.5, 5); // move slower to pose

        //robot.arm.open_claw(); // deposit sample in basket
    }

    public void sampleIntake() {
        //robot.lift.elevate_to(LiftConst.INIT);
        //robot.arm.waiting_position();
        robot.drive_train.odo_move(samplePose(nextSample), 0.75, 5);

        /*robot.intake.intake_intake();
        while (!robot.intake.current_color_name.equals("YELLOW")) {
            robot.intake.intake_run_auto(1);
            if (robot.reach.position < 200) {
                robot.reach.extend_to(robot.reach.position + 10);
                halt(0.1);
            }
        }*/

        halt(2);

        //robot.reach.min_reach();
    }

    public Pose samplePose(int pose) {
        if (pose == AutoConst.SAMPLE_PARTNER) {
            nextSample = AutoConst.SAMPLE_RIGHT; // usually samplePartner is first followed by sampleRight
            return AutoConst.partnerSamplePose;

        } else if (pose == AutoConst.SAMPLE_MID) {
            nextSample = AutoConst.SAMPLE_LEFT; // followed by sampleLeft
            return AutoConst.midSamplePose;

        } else if (pose == AutoConst.SAMPLE_LEFT) {
            nextSample = 0; // nothing is after this so it wont do anything ending the loop
            return AutoConst.leftSamplePose;

        } else {
            nextSample = AutoConst.SAMPLE_MID; // followed by sampleMid
            return AutoConst.rightSamplePose;
        }

    }

    public void subPark() {
        Pose start = robot.drive_train.getCurrentPose();
        robot.drive_train.odo_drive_towards(start.x, AutoConst.subPark.y, AutoConst.subPark.a, 1);

        halt(1);

        //robot.arm.basket_position(); // for the robot to touch the low rung
        robot.drive_train.odo_move(AutoConst.subPark, 1, 2);
    }
}