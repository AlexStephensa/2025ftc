package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MID_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.UNIT_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.MIN_LENGTH;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.TWEAK_MAX_ADD;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.SECTION;
import static org.firstinspires.ftc.teamcode.components.live.ReachConfig.SECT_LENGTH;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

//@Config
class ReachConfig {
    public static final int UNIT_LENGTH     = 10;   // mm
    public static final int MAX_LENGTH      = 600;  // mm
    public static final int MIN_LENGTH      = 150;  // mm
    public static final int MID_LENGTH      = 400; // mm
    public static final int TWEAK_MAX_ADD   = 20;   // mm
    public static final int SECTION         = 6;    // number of scissor sections
    public static final int SECT_LENGTH     = 112;  // length of section-arms in mm (from end to end)

}

public class Reach extends Component {
    //// SERVOS ////
    public ServoQUS reach_l;
    public ServoQUS reach_r;

    //// VARIABLES ////
    public int position;    // position stored in levels
    private boolean starting_move = false;
    public int reach_l_target = MIN_LENGTH;
    public int reach_r_target = MIN_LENGTH;
    public double reach_l_angle = reach_angle(MIN_LENGTH);
    public double reach_r_angle = reach_angle(MIN_LENGTH);

    static double tweak = 0;
    static double tweak_cache = 0;

    {
        name = "Reach";
    }

    public Reach(Robot robot) { super(robot); }

    //@Override
    public void registerHardware (HardwareMap hwmap) {
        //// SERVOS ////
        reach_l     = new ServoQUS(hwmap.get(Servo.class, "reachL"));
        reach_r     = new ServoQUS(hwmap.get(Servo.class, "reachR"));

    }
    //@Override
    public void update(OpMode opmode) {
        super.update(opmode);
        /*if (starting_move) {
            if (position == 0) {
                reach_l.queue_position(Math.toDegrees(reach_angle(MIN_LENGTH)));
                reach_r.queue_position(Math.toDegrees(reach_angle(MIN_LENGTH)));
                update_reach();
            } else {
                reach_l.queue_position(Math.toDegrees(reach_angle(reach_l_target)));
                reach_r.queue_position(Math.toDegrees(reach_angle(reach_r_target)));
                update_reach();
            }
            starting_move = false;
        }

         */
        update_reach();
        if (tweak != tweak_cache) {
            tweak_cache = tweak;
            reach_l.queue_position(
                    reach_angle(Range.clip(
                            reach_l_target + (int) (tweak * UNIT_LENGTH / 2),
                            MIN_LENGTH,
                            MAX_LENGTH
                    ))
            );
            reach_r.queue_position(
                    reach_angle(Range.clip(
                            reach_r_target + (int) (tweak * UNIT_LENGTH),
                            MIN_LENGTH,
                            MAX_LENGTH + TWEAK_MAX_ADD
                    ))
            );
        }
    }
    //@Override
    public void startup() {
        super.startup();
        update_reach();
    }
    public void shutdown() {
        //shut down
    }
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("LR TARGET",TELEMETRY_DECIMAL.format(reach_l_target));
        telemetry.addData("RR TARGET",TELEMETRY_DECIMAL.format(reach_r_target));
        telemetry.addData("LR ANGLE (RAD)", TELEMETRY_DECIMAL.format(reach_l_angle));
        telemetry.addData("RR ANGLE (RAD)", TELEMETRY_DECIMAL.format(reach_r_angle));
        telemetry.addData("LR ANGLE", TELEMETRY_DECIMAL.format(Math.toDegrees(reach_l_angle) / 360));
        telemetry.addData("RR ANGLE", TELEMETRY_DECIMAL.format(Math.toDegrees(reach_r_angle) / 360));
        telemetry.addData("REACH MOVING", starting_move);
        //telemetry.addData("REACH", TELEMETRY_DECIMAL.format());
    }

    private double reach_angle(double dis) {
        return Math.acos(dis / (SECTION * SECT_LENGTH));
    }
    private void update_reach() {
        reach_l.queue_position((Math.toDegrees(reach_l_angle)) / 360);
        reach_r.queue_position((Math.toDegrees(reach_r_angle)) / 360);

        reach_l.update();
        reach_r.update();
    }
    public void extend_to(int target) {
        position = Range.clip(target, MIN_LENGTH, MAX_LENGTH);
        reach_l_target = (position);
        reach_r_target = (position);
        reach_l_angle = reach_angle(reach_l_target);
        reach_r_angle = reach_angle(reach_r_target);
        starting_move = true;
    }
    public void min_reach() {
        extend_to(MIN_LENGTH);
    }
    public void max_reach() {
        extend_to(MAX_LENGTH);
    }
    public void mid_reach() {
        extend_to(MID_LENGTH);
    }
    public void tweak(double tweak) {
        Reach.tweak = tweak;
    }
}