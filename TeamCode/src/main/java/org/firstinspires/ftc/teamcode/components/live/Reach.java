package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MAX_LEVEL;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.UNIT_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.REACH_OFFSET;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MIN_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.TWEAK_MAX_ADD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;

//@Config

class ReachConfig {
    public static int UNIT_LENGTH = 1; // in cm
    public static int REACH_OFFSET = 0;
    public static int MAX_LENGTH = 12;
    public static int MIN_LENGTH = 0;

    public static int TWEAK_MAX_ADD = 100;
}

public class Reach extends Component {
    //// SERVOS ////
    public ServoQUS reach_l;
    public ServoQUS reach_r;

    //// SENSORS ////
    public DigitalChannel limit_switchH;
    public int position;

    private boolean starting_move = false;

    public int reach_l_target = 0;
    public int reach_r_target = 0;

    public int reach_l_offset = 0;
    public int reach_r_offset = 0;

    static double tweak = 0;
    static double tweak_cache = 0;
    public static int max_position = MAX_LENGTH;

    {
        name = "Reach";
    }

    public Reach(Robot robot)
    {
        super(robot);
    }

    //@Override
    public void registerHardware (HardwareMap hwmap) {
        //// SERVOS ////
        reach_l     = new ServoQUS(hwmap.get(Servo.class, "horiServL"));
        reach_r     = new ServoQUS(hwmap.get(Servo.class, "horiServR"));

        limit_switchH = hwmap.get(DigitalChannel.class, "limit_switchV");
    }


    //@Override
    public void update(OpMode opmode) {
        super.update(opmode);

        if (starting_move) {
            if ((position == 0) || (position == -1)) {
                if(limit_switchH.getState()) {
                    reach_l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    reach_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    set_power(-1);
                }
            } else {
                reach_l.setTargetPosition(reach_l_target+reach_l_offset);
                reach_r.setTargetPosition(reach_r_target+reach_r_offset);
                reach_l.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                reach_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                set_power(1);
            }

            starting_move = false;
        }

        if (((position == 0) || (position == -1)) && (reach_l.getPower() == -1 || reach_r.getPower() == -1)) {
            if ((limit_switchH.getState()) && (reach_l.getPower() != 0) && (reach_r.getPower() != 0)) {
                reach_l_offset = reach_l.getCurrentPosition();
                reach_l.setPower(0);
                reach_r = reach_r.getCurrentPosition();
                reach_r.setPower(0);
            }
        }

        if (tweak != tweak_cache) {
            tweak_cache = tweak;
            reach_l.setTargetPosition(
                    Range.clip(
                            reach_l_target + reach_l_offset + (int) (tweak * UNIT_LENGTH / 2),
                            MIN_LENGTH * UNIT_LENGTH,
                            MAX_LENGTH * UNIT_LENGTH+TWEAK_MAX_ADD
                    )
            );
            reach_r.setTargetPosition(
                    Range.clip(
                            reach_r_target + reach_r_offset + (int) (tweak * UNIT_LENGTH),
                            MIN_LENGTH * UNIT_LENGTH,
                            MAX_LENGTH * UNIT_LENGTH
                    )
            );
        }
    }

    //@Override
    public void startup() {
        //super.startup();

    }

    public void shutdown() {
        //set_power(0);

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("LR TURNS",TELEMETRY_DECIMAL.format(reach_l.getCurrentPosition()));
        telemetry.addData("RR TURNS",TELEMETRY_DECIMAL.format(reach_r.getCurrentPosition()));

        telemetry.addData("LR TARGET",TELEMETRY_DECIMAL.format(reach_l_target));
        telemetry.addData("RR TARGET",TELEMETRY_DECIMAL.format(reach_r_target));

        telemetry.addData("LR OFFSET", TELEMETRY_DECIMAL.format(reach_l_offset));
        telemetry.addData("RR OFFSET", TELEMETRY_DECIMAL.format(reach_r_offset));

        telemetry.addData("LIFT BUSY",reach_l.isBusy()+" "+reach_r.isBusy());

        telemetry.addData("REACH RUNNING", running_lift());

        telemetry.addData("LIM", !limit_switchH.getState());

    }

    public void set_power(double speed) {
        reach_l.setPower(speed);
        reach_r.setPower(speed);
    }

    public boolean running_lift() {
        return reach_l.getPower() != 0 || reach_r.getPower() != 0;
    }

    private void set_target_position(int pos) {
        reach_l_target = pos;
        reach_r_target = pos;
    }

    public void extend(int amt) {
        elevate_to(position + amt);
    }

    public void elevate_to(int target) {
        position = Math.max(Math.min(target, MAX_LEVEL), MIN_LENGTH);
        set_target_position((position * UNIT_LENGTH) + REACH_OFFSET);
        starting_move = true;
    }

    public void min_lift() {
        extend(MIN_LENGTH - position);
    }

    public void max_lift() {
        extend(MAX_LEVEL - position);
    }

    public void elevate_without_stops(int amt) {
        position = position + amt;
        set_target_position((position * UNIT_LENGTH) + REACH_OFFSET);
        starting_move = true;
    }
    public void tweak(double tweak) {
        this.tweak = tweak;
    }

}