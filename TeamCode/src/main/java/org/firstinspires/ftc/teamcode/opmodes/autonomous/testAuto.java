package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.AutoConst;
import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testAuto", group = "autonomous")
public class testAuto extends LiveAutoBase {

    double drive_speed = 0.5;

    public int nextSample = 0; // what part of the sample cycle the robot is on

    @Override
    public void on_init() {
        robot.arm.transfer_position();
        robot.arm.close_claw();
        robot.drive_train.odo_reset(AutoConst.initPose);
    }

    @Override
    public void on_start() {

        highBasket();
        sampleIntake(AutoConst.SAMPLE_RIGHT);

    }

    @Override
    public void on_stop() {
    }

    public void highBasket() {
        robot.drive_train.odo_drive_towards(AutoConst.highBasketPose, 1); // quickly driving to pose

        robot.lift.elevate_to(LiftConst.HIGH_BASKET);
        robot.arm.basket_position(); // moving lift and arm to deposit positions

        halt(0.5); // wait for the robot to approach pose

        robot.drive_train.odo_move(AutoConst.highBasketPose, 0.5, 1); // move slower to pose

        robot.arm.open_claw(); // deposit sample in basket
    }

    public void lowBasket() {
        robot.drive_train.odo_drive_towards(AutoConst.lowBasketPose, 1); // quickly driving to pose

        robot.lift.elevate_to(LiftConst.LOW_BASKET);
        robot.arm.basket_position(); // moving lift and arm to deposit positions

        halt(0.5); // wait for the robot to approach pose

        robot.drive_train.odo_move(AutoConst.lowBasketPose, 0.5, 1); // move slower to pose

        robot.arm.open_claw(); // deposit sample in basket
    }

    public void sampleIntake(int type) {
        robot.lift.elevate_to(LiftConst.INIT);
        robot.arm.waiting_position();
        if (type != 0) {
            if (type == AutoConst.SAMPLE_RIGHT) {
                robot.drive_train.odo_move(AutoConst.rightSamplePose, 0.5, 1.5);
                nextSample = 2;
            } else if (type == AutoConst.SAMPLE_MID) {
                robot.drive_train.odo_move(AutoConst.midSamplePose, 0.5, 1.5);
                nextSample = 3;
            } else if (type == AutoConst.SAMPLE_LEFT) {
                robot.drive_train.odo_move(AutoConst.leftSamplePose, 0.5, 1.5);
                nextSample = 0;
            } else if (type == AutoConst.SAMPLE_PARTNER) {
                robot.drive_train.odo_move(AutoConst.partnerSamplePose, 0.5, 1.5);
                nextSample = 1; // usually samplePartner is first followed by sampleRight...
            }
        }
    }

    public void subPark() {
        Pose start = robot.drive_train.getCurrentPose();
        robot.drive_train.odo_drive_towards(AutoConst.subPark.x, start.y, AutoConst.subPark.a, 1);

        halt(1);

        robot.drive_train.odo_drive_towards(AutoConst.subPark, 1);
        halt(1);


    }


}