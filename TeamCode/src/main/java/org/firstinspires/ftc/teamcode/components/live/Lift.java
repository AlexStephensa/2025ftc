package org.firstinspires.ftc.teamcode.components.live;

import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.LIFT_LEVELS;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.LIFT_OFFSET;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MAX_LEVEL;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MIN_LEVEL;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_D;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_I;
import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.PID_P;

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
    public static int MAX_LEVEL = 5; //highest level the virtual robot can extend
    public static int MIN_LEVEL = 0;

    public static int[] LIFT_LEVELS = {
            0,     // level
            6160,  // high bin
            2440,  // low bin
            2140,  // specimen
            5180,  // high rung
            6220   // hang
    };

    public static double PID_P = 1;
    public static double PID_I = 0;
    public static double PID_D = 0;
}

public class Lift extends Component {
    private PIDFController pid_control;

    public int max_level = LiftConfig.MAX_LEVEL;

    //// MOTORS ////
    public DcMotorEx lift_l;
    public DcMotorEx lift_r;

    //// SENSORS ////
    public DigitalChannel limit_switchV;

    public int level;

    public int lift_target = 0;

    public int lift_l_offset = 0;
    public int lift_r_offset = 0;

    static double tweak = 0;

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
        lift_l     = hwmap.get(DcMotorEx.class, "liftF");
        lift_r     = hwmap.get(DcMotorEx.class, "liftB");

        limit_switchV = hwmap.get(DigitalChannel.class, "vLimSwitch");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        pid_control.setTargetPosition(lift_target);
        pid_speed = pid_control.update(lift_l.getCurrentPosition()) / 100;

        set_power(pid_speed);
    }

    @Override
    public void startup() {
        super.startup();

        lift_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutdown() {
        set_power(0);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("TURNS",TELEMETRY_DECIMAL.format(lift_l.getCurrentPosition()));

        telemetry.addData("TARGET",TELEMETRY_DECIMAL.format(lift_target));

        telemetry.addData("LL OFFSET", TELEMETRY_DECIMAL.format(lift_l_offset));
        telemetry.addData("RL OFFSET", TELEMETRY_DECIMAL.format(lift_r_offset));

        telemetry.addData("LEVEL", level);

        telemetry.addData("LIM", !limit_switchV.getState());

        telemetry.addData("PID VEL", pid_speed);

    }

    public void set_power(double speed) {
        lift_l.setPower(speed);
        lift_r.setPower(-speed);
    }

    private void set_target_position(int pos) {
        lift_target = pos;
    }

    public void elevate(int amt) {
        elevate_to(level + amt);
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