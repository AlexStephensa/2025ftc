package org.firstinspires.ftc.teamcode.components.live;

import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.AT_THRESH;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.LIFT_LEVELS;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.LIFT_OFFSET;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MAX_LEVEL;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MIN_LEVEL;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_D;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_I;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_P;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.THRESHOLD;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Config
class LiftConfig {
    public static int LIFT_OFFSET = 0;
    public static int MAX_LEVEL = 5; // highest level the virtual robot can extend
    public static int MIN_LEVEL = 0;

    public static int MAX_EXTENSION = 72000; // highest safe extension of physical robot

    public static int MIN_LIFT_OVERSHOOT = 2000;
    public static int THRESHOLD = 200;
    public static boolean AT_THRESH = true;

    public static int TWEAK_MAX_ADD = 1000;

    public static int[] LIFT_LEVELS = {
            0,      // level 0
            70240,  // high bin
            30350,  // low bin
            24240,  // specimen
            57520,  // high rung
            68200   // hang
    };

    public static double PID_P = 0.08;
    public static double PID_I = 0.0016;
    public static double PID_D = 0.0002;
}

public class Lift extends Component {
    private PIDFController pid_control;

    public int max_level = LiftConfig.MAX_LEVEL;

    //// MOTORS ////
    public DcMotorEx lift_f;
    public DcMotorEx lift_b;

    //// SENSORS ////
    public DigitalChannel limit_switchL;


    public int level;
    public int lift_target = 0;
    public int lift_offset = 0;

    private boolean last_limit_switch = true;

    static double tweak = 0;
    static double tweak_cache = 0;

    static double pid_speed = 0;

    {
        name = "Lift";
    }

    public Lift(Robot robot)
    {
        super(robot);

        pid_control = new PIDFController(new PIDCoefficients(PID_P, PID_I, PID_D));
        pid_control.setOutputBounds(-100, 100);
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// MOTORS ////
        lift_f = hwmap.get(DcMotorEx.class, "liftF");
        lift_b = hwmap.get(DcMotorEx.class, "liftB");

        limit_switchL = hwmap.get(DigitalChannel.class, "vLimSwitch");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        boolean cur_limit_switch = !limit_switchL.getState();
        int cur_position = lift_f.getCurrentPosition();

        if (lift_target == 0) {
            if (cur_limit_switch && !last_limit_switch) {
                lift_offset = cur_position;
            }

            if (!cur_limit_switch) {
                pid_control.setTargetPosition(lift_offset - LiftConfig.MIN_LIFT_OVERSHOOT);
            } else {
                pid_control.setTargetPosition(lift_offset);
            }
        } else {
            pid_control.setTargetPosition(lift_offset + lift_target);

             /*if (tweak != tweak_cache) {
                 tweak_cache = tweak;
                 pid_control.setTargetPosition(
                         Range.clip(
                                 lift_target + lift_offset + (int) (tweak * TWEAK_MAX_ADD),
                                 0,
                                 MAX_EXTENSION + TWEAK_MAX_ADD
                         )
                 );
             }*/
        }

        pid_speed = pid_control.update(cur_position) / 100;

        // set_power(hold_threshold(pid_speed));
        set_power(pid_speed);

        last_limit_switch = cur_limit_switch;
    }

    @Override
    public void startup() {
        super.startup();

        lift_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutdown() {
        set_power(0);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("TURNS",TELEMETRY_DECIMAL.format(lift_f.getCurrentPosition()));

        telemetry.addData("TARGET",TELEMETRY_DECIMAL.format(lift_target));

        telemetry.addData("OFFSET", TELEMETRY_DECIMAL.format(lift_offset));

        telemetry.addData("LEVEL", level);

        telemetry.addData("LIFT LIM", !limit_switchL.getState());

        telemetry.addData("PID VEL", pid_speed);

        //telemetry.addData("AT THRESH", AT_THRESH);

    }

    public void set_power(double speed) {
        lift_f.setPower(speed);
        lift_b.setPower(-speed);
    }

    private void set_target_position(int pos) {
        lift_target = pos;
    }

    public void elevate(int amt) {
        elevate_to(level + amt);
    }

    public double hold_threshold(double speed){
        if (Math.abs(lift_target - lift_f.getCurrentPosition()) <= THRESHOLD) {
            AT_THRESH = true;
            return 0;
        } else {
            AT_THRESH = false;
            return speed;
        }
    }

    public void elevate_to(int target_level) {
        level = Math.max(Math.min(target_level, MAX_LEVEL), MIN_LEVEL);
        set_target_position((LIFT_LEVELS[level]) + LIFT_OFFSET);
    }

    public void min_lift() {
        elevate(MIN_LEVEL - level);
    }

    public void max_lift() {
        elevate(MAX_LEVEL - level);
    }

    public void tweak(double tweak) {
        this.tweak = tweak;
    }
}