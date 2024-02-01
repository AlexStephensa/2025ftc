package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.components.Component;

import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.*;

// Elevator lifts the stone and extender up
// Extender extends over the tower, and the grabber releases the stone


//@Config

class LiftConfig {
    public static int BLOCK_HEIGHT = 240; //In encoder counts
    public static int LIFT_OFFSET = 0;
    public static int MAX_LEVEL = 12;
    public static int MIN_LEVEL = 0;

    public static double PID_P = 15;
    public static double PID_I = 0.1;
    public static double PID_D = 4;

    public static int LIFT_DOWN_OVERSHOOT = 100;

    public static int TWEAK_MAX_ADD = 100;

}

public class Lift extends Component {

    //// MOTORS ////
    public DcMotorEx lift_l;
    public DcMotorEx lift_r;

    //// SENSORS ////
    public DigitalChannel limit_switch;

    public int level;

    private boolean starting_move = false;

    public int lift_l_target = 0;
    public int lift_r_target = 0;

    public int lift_l_offset = 0;
    public int lift_r_offset = 0;

    static double ext_pos = 0;
    static double ext_pos_cache = 0;

    static double grab_pos = 0;
    static double grab_pos_cache = 0;

    static double turn_pos = 0;
    static double turn_pos_cache = 0;

    static double cap_pos = 0;
    static double cap_pos_cache = 0;

    static double tweak = 0;
    static double tweak_cache = 0;
    public static int max_level = MAX_LEVEL;

    {
        name = "Lift";
    }

    public Lift(Robot robot)
    {
        super(robot);
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// MOTORS ////
        lift_l     = hwmap.get(DcMotorEx.class, "lift_l");
        lift_r     = hwmap.get(DcMotorEx.class, "lift_r");

        limit_switch = hwmap.get(DigitalChannel.class, "limit_switch");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        if (starting_move) {
            if ((level == 0) || (level == -1)) {
                if(limit_switch.getState()) {
                    lift_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    set_power(-1);
                }
            } else {
                lift_l.setTargetPosition(lift_l_target+lift_l_offset);
                lift_r.setTargetPosition(lift_r_target+lift_r_offset);
                lift_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                set_power(1);
            }

            starting_move = false;
        }

        if (((level == 0) || (level == -1)) && (lift_l.getPower() == -1 || lift_r.getPower() == -1)) {
            if ((limit_switch.getState()) && (lift_l.getPower() != 0) && (lift_r.getPower() != 0)) {
                lift_l_offset = lift_l.getCurrentPosition();
                lift_l.setPower(0);
                lift_r_offset = lift_r.getCurrentPosition();
                lift_r.setPower(0);
            }
        }

        if (tweak != tweak_cache) {
            tweak_cache = tweak;
            lift_l.setTargetPosition(
                    Range.clip(
                            lift_l_target + lift_l_offset + (int) (tweak * BLOCK_HEIGHT / 2),
                            MIN_LEVEL*BLOCK_HEIGHT,
                            MAX_LEVEL*BLOCK_HEIGHT+TWEAK_MAX_ADD
                    )
            );
            lift_r.setTargetPosition(
                    Range.clip(
                            lift_r_target + lift_r_offset + (int) (tweak * BLOCK_HEIGHT),
                            MIN_LEVEL*BLOCK_HEIGHT,
                            MAX_LEVEL*BLOCK_HEIGHT
                    )
            );
        }
    }

    @Override
    public void startup() {
        super.startup();

        lift_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDCoefficients pid_coeffs = new PIDCoefficients(PID_P, PID_I, PID_D);

        lift_l.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid_coeffs);
        lift_r.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pid_coeffs);

        lift_l.setDirection(DcMotorSimple.Direction.REVERSE);

        lift_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shutdown() {
        set_power(0);
        lift_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("LL TURNS",TELEMETRY_DECIMAL.format(lift_l.getCurrentPosition()));
        telemetry.addData("RL TURNS",TELEMETRY_DECIMAL.format(lift_r.getCurrentPosition()));

        telemetry.addData("LL TARGET",TELEMETRY_DECIMAL.format(lift_l_target));
        telemetry.addData("RL TARGET",TELEMETRY_DECIMAL.format(lift_r_target));

        telemetry.addData("LL OFFSET", TELEMETRY_DECIMAL.format(lift_l_offset));
        telemetry.addData("RL OFFSET", TELEMETRY_DECIMAL.format(lift_r_offset));

        telemetry.addData("LIFT BUSY",lift_l.isBusy()+" "+lift_r.isBusy());

        telemetry.addData("LIFT RUNNING", running_lift());

        telemetry.addData("LEVEL", level);

        telemetry.addData("LIM", !limit_switch.getState());

    }

    public void set_power(double speed) {
        lift_l.setPower(speed);
        lift_r.setPower(speed);
    }

    public boolean running_lift() {
        return lift_l.getPower() != 0 || lift_r.getPower() != 0;
    }

    private void set_target_position(int pos) {
        lift_l_target = pos;
        lift_r_target = pos;
    }

    public void elevate(int amt) {
        elevate_to(level + amt);
    }

    public void elevate_to(int target) {
        level = Math.max(Math.min(target, MAX_LEVEL), MIN_LEVEL);
        set_target_position((level * BLOCK_HEIGHT) + LIFT_OFFSET);
        starting_move = true;
    }

    public void min_lift() {
        elevate(MIN_LEVEL - level);
    }

    public void max_lift() {
        elevate(MAX_LEVEL - level);
    }

    public void elevate_without_stops(int amt) {
        level = level + amt;
        set_target_position((level * BLOCK_HEIGHT) + LIFT_OFFSET);
        starting_move = true;
    }
    public void tweak(double tweak) {
        this.tweak = tweak;
    }
}