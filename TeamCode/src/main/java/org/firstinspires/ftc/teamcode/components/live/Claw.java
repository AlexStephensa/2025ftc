package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class ClawConfig {
    public static double CLAW_L_OPEN = 0.39;
    public static double CLAW_R_OPEN = 0.45;
    public static double CLAW_L_CLOSE = 0.33;
    public static double CLAW_R_CLOSE = 0.51;
    public static double CLAW_PITCH_UP = 0.6;
    public static double CLAW_PITCH_DOWN = 0.345;
    public static double MIN_DISTANCE = 25; //mm
}

public class Claw extends Component {

    public ServoQUS claw_pitch;
    public ServoQUS claw_l;
    public ServoQUS claw_r;
    public Rev2mDistanceSensor left_ds;
    public Rev2mDistanceSensor right_ds;

    public double left_distance = 1000;
    public double right_distance = 1000;

    public boolean left_claw_open = false;
    public boolean right_claw_open = false;

    {
        name = "Claw";
    }

    public Claw(Robot robot) {
        super(robot);
    }

    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        claw_pitch = new ServoQUS(hwmap.get(Servo.class, "claw_pitch"));
        claw_l = new ServoQUS(hwmap.get(Servo.class, "claw_l"));
        claw_r = new ServoQUS(hwmap.get(Servo.class, "claw_r"));
        left_ds = hwmap.get(Rev2mDistanceSensor.class, "left_ds");
        right_ds = hwmap.get(Rev2mDistanceSensor.class, "right_ds");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        claw_pitch.update();
        claw_l.update();
        claw_r.update();

        if (robot.cycle % 10 == 0) {
            left_distance = left_ds.getDistance(DistanceUnit.MM);
        }

        if (robot.cycle % 10 == 5) {
            right_distance = right_ds.getDistance(DistanceUnit.MM);
        }
    }

    @Override
    public void startup() {
        super.startup();

        close_left();
        close_right();
        pitch_down();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("L DIST", TELEMETRY_DECIMAL.format(left_distance));
        telemetry.addData("R DIST", TELEMETRY_DECIMAL.format(right_distance));
    }

    public void close_left() {
        claw_l.queue_position(ClawConfig.CLAW_L_CLOSE);
        left_claw_open = false;
    }

    public void close_right() {
        claw_r.queue_position(ClawConfig.CLAW_R_CLOSE);
        right_claw_open = false;
    }

    public void open_left() {
        claw_l.queue_position(ClawConfig.CLAW_L_OPEN);
        left_claw_open = true;
    }

    public void open_right() {
        claw_r.queue_position(ClawConfig.CLAW_R_OPEN);
        right_claw_open = true;
    }

    public void open() {
        open_left();
        open_right();
    }

    public void pitch_up() {
        claw_pitch.queue_position(ClawConfig.CLAW_PITCH_UP);
    }

    public void pitch_down() {
        claw_pitch.queue_position(ClawConfig.CLAW_PITCH_DOWN);
    }

    public boolean left_detected() {
        return left_distance < ClawConfig.MIN_DISTANCE;
    }

    public boolean right_detected() {
        return right_distance < ClawConfig.MIN_DISTANCE;
    }
}
