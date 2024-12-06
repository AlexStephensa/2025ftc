package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.live.Lift;
import org.firstinspires.ftc.teamcode.opmodes.LiveTeleopBase;

@TeleOp(name="Teleop Live", group="driver control")
//@Disabled
public class LiveTeleop extends LiveTeleopBase {

    boolean dpad1_up_pressed = false;
    boolean dpad1_down_pressed = false;
    boolean dpad1_left_pressed = false;
    boolean dpad1_right_pressed = false;

    boolean gp1_a_pressed = false;
    boolean gp1_b_pressed = false;
    boolean gp1_y_pressed = false;

    int prepared_level = 1;
    int prepared_cone = 4;


    @Override
    public void on_init() {
    }

    @Override
    public void on_start() {
        this.getRuntime();
    }

    @Override
    public void on_loop() {
        /*if (!robot.claw.left_claw_open && !robot.claw.right_claw_open) {
            gamepad1.setLedColor(0, 1, 0, LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 1, 0, LED_DURATION_CONTINUOUS);
        }
        else {
            gamepad1.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, LED_DURATION_CONTINUOUS);
        }*/

        /// GAMEPAD TWO BACK HOTKEYS ///
        if(gamepad2.back) {
            if(gamepad2.y) {
                robot.Reach.max_reach();
            }
            else {
                robot.Reach.min_reach();
            }

        } else {

            /// LIFT ///
            if(gamepad2.x){
                robot.lift.elevate_to(prepared_level);
            }

            // Intake
            robot.intake.intakeRun(gamepad2.right_trigger - gamepad2.left_trigger);

            if(gamepad2.dpad_left) {
                robot.intake.intake_intake();
            }

            if(gamepad2.dpad_right) {
                robot.intake.intake_cradel();
            }

            if(gamepad2.dpad_up) {
                robot.intake.intake_init();
            }

            if(gamepad2.dpad_down) {
                robot.intake.intake_transfer();
            }

            /*f (gamepad2.left_bumper) {
                robot.intake.intake_cradel();
            }

            if (gamepad2.right_bumper){
                robot.intake.();
            }*/

            if(gamepad2.dpad_up && !dpad1_up_pressed) {
                prepared_level = Range.clip(prepared_level + 1, 0, Lift.max_level);
                dpad1_up_pressed = true;
            } else if (!gamepad2.dpad_up) {
                dpad1_up_pressed = false;
            }

            if(gamepad2.dpad_down && !dpad1_down_pressed) {
                prepared_level = Range.clip(prepared_level - 1, 0, Lift.max_level);
                dpad1_down_pressed = true;
            } else if (!gamepad2.dpad_down) {
                dpad1_down_pressed = false;
            }

            robot.Reach.tweak(gamepad2.right_trigger - gamepad2.left_trigger);

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
        );                      // Turn speed modifier ^^

    }

    @Override
    public void on_stop(){

    }
}