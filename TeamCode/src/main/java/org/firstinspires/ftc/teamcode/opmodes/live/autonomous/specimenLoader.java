package org.firstinspires.ftc.teamcode.opmodes.live.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.live.LiveAutoBase;

@Autonomous(name = "specimenLoader", group = "autonomous", preselectTeleOp = "Teleop Live")
public class specimenLoader extends LiveAutoBase {

    AutoSpecimen auto;
    double gridMod = Math.PI * 3/2;
    @Override
    public void on_init() {
        auto = new AutoSpecimen(robot, 0.66, 0.5);
        auto.specimenInit();
    }

    @Override
    public void on_start() {
        robot.drive_train.odo_drive(108, 8, Math.PI * 3/2 + gridMod, auto.fast);
    }

    @Override
    public void on_stop() {

    }
}