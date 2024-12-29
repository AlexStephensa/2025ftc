package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.LiveTeleopBase;

@TeleOp(name="Teleop Live", group="driver control")
//@Disabled
public class LiveTeleop extends LiveTeleopBase {

    boolean dpad1_up_pressed = false;
    boolean dpad1_down_pressed = false;
    boolean gp1_back_pressed = false;

    boolean intake = false;

    int prepared_level = 1;


    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        this.getRuntime();

        robot.arm.open_claw();
    }

    @Override
    public void on_loop() {
        // Reach

        if(gamepad1.back && !gp1_back_pressed) {
            robot.intake.toggle_wanted_color();
            gamepad1.setLedColor(robot.intake.intake_color_wanted == "RED" ? 1 : 0, 0, robot.intake.intake_color_wanted == "BLUE" ? 1 : 0, LED_DURATION_CONTINUOUS);
            gp1_back_pressed = true;
        } else if (!gamepad1.back) {
            gp1_back_pressed = false;
        }

        if(gamepad2.dpad_right) {
            robot.reach.max_reach();
        }
        else if (gamepad2.dpad_left){
            robot.reach.min_reach();
            robot.intake.intake_transfer();
        }

        robot.intake.intake_run(gamepad2.a ? 1 : (gamepad2.b ? -1 : 0), gamepad1, gamepad2);

        if(gamepad2.right_bumper) {
            robot.intake.intake_intake();
            intake = true;
        }

        robot.reach.tweak(gamepad2.right_trigger - gamepad2.left_trigger);

        if(gamepad2.back) {
            robot.lift.min_lift();
        } else {
            /// LIFT ///
            if(gamepad2.x){
                robot.lift.elevate_to(prepared_level);
            }

            if(gamepad2.dpad_up && !dpad1_up_pressed) {
                prepared_level = Range.clip(prepared_level + 1, 0, robot.lift.max_level);
                dpad1_up_pressed = true;
            } else if (!gamepad2.dpad_up) {
                dpad1_up_pressed = false;
            }

            if(gamepad2.dpad_down && !dpad1_down_pressed) {
                prepared_level = Range.clip(prepared_level - 1, 0, robot.lift.max_level);
                dpad1_down_pressed = true;
            } else if (!gamepad2.dpad_down) {
                dpad1_down_pressed = false;
            }
        }

        /// DRIVE CONTROLS ///
        double speed_mod = 1;

        if(gamepad1.left_bumper) {
            speed_mod = 0.5;
        } else if(gamepad1.right_bumper) {
            speed_mod = 0.25;
        }
            // Change +\- here for motor directions
        robot.drive_train.mecanum_drive(
            (gamepad1.left_stick_x) * speed_mod,
            (gamepad1.left_stick_y) * speed_mod,
            (gamepad1.right_stick_x) * speed_mod * 0.85
        );
    }

    @Override
    public void on_stop(){

    }
}