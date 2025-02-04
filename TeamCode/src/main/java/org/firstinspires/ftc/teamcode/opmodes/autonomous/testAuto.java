package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.LiftConst;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.opmodes.LiveAutoBase;

@Autonomous(name = "testAuto", group = "autonomous")
public class testAuto extends LiveAutoBase {

    double drive_speed = 0.5;

    @Override
    public void on_init() {
        robot.arm.transfer_position();
        robot.arm.close_claw();
        robot.drive_train.odo_reset(32.5, 8.5, Math.PI/2);
    }

    @Override
    public void on_start() {

        highBasket();

    }

    @Override
    public void on_stop() {
    }

    private void highBasket() {
        Pose highBasket = new Pose(17, 17 , (Math.PI * 5/4));
        robot.lift.elevate_to(LiftConst.HIGH_BASKET);
        robot.arm.basket_position();
        robot.drive_train.odo_move(highBasket, drive_speed, 1.5);
        robot.arm.open_claw();

    }
}