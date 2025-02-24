package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class ArmConfig {
    static public double ELBOW_L_TRANSFER_POSITION = 0.56;
    static public double ELBOW_L_WAITING_POSITION = 0.52;
    static public double ELBOW_L_SPECIMEN_POSITION = 0.88;
    static public double ELBOW_L_BASKET_POSITION = 0.15;
    static public double ELBOW_L_PARK_POSITION = 0.18;

    static public double ELBOW_R_TRANSFER_POSITION = 0.56;
    static public double ELBOW_R_WAITING_POSITION = 0.52;
    static public double ELBOW_R_SPECIMEN_POSITION = 0.88;
    static public double ELBOW_R_BASKET_POSITION = 0.15;
    static public double ELBOW_R_PARK_POSITION = 0.18;

    static public double WRIST_TRANSFER_POSITION = 0.67;
    static public double WRIST_SPECIMEN_POSITION = 0.75;
    static public double WRIST_BASKET_POSITION = 0.15;
    static public double WRIST_PARK_POSITION = 0.4;

    static public double CLAW_CLOSE_POSITION = 0.72;
    static public double CLAW_OPEN_POSITION = 0.4;
}

public class Arm extends Component {
    private String arm_current;
    private boolean claw_state;

    //// SERVOS ////
    public ServoQUS elbow_l;
    public ServoQUS elbow_r;
    public ServoQUS wrist;
    public ServoQUS claw;

    {
        name = "Claw";
    }

    public Arm(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap) {
        //// SERVOS ////
        elbow_l     = new ServoQUS(hwmap.get(Servo.class, "elbow_l"));
        elbow_r     = new ServoQUS(hwmap.get(Servo.class, "elbow_r"));
        elbow_r.servo.setDirection(Servo.Direction.REVERSE);

        wrist     = new ServoQUS(hwmap.get(Servo.class, "wrist"));

        claw     = new ServoQUS(hwmap.get(Servo.class, "claw"));
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        elbow_l.update();
        elbow_r.update();
        wrist.update();
        claw.update();

    }

    @Override
    public void startup() {
        waiting_position();
        open_claw();
    }

    @Override
    public void shutdown() {}

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("ARM CURRENT", arm_current);
        telemetry.addData("CLAW CLOSED", claw_state);
    }

    public void transfer_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_TRANSFER_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_TRANSFER_POSITION);
        wrist.queue_position(ArmConfig.WRIST_TRANSFER_POSITION);
        arm_current = "TRANS";
    }

    public void waiting_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_WAITING_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_WAITING_POSITION);
        wrist.queue_position(ArmConfig.WRIST_TRANSFER_POSITION);
        open_claw();
        arm_current = "WAIT";
    }

    public void basket_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_BASKET_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_BASKET_POSITION);
        wrist.queue_position(ArmConfig.WRIST_BASKET_POSITION);
        arm_current = "BASK";
    }

    public void specimen_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_SPECIMEN_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_SPECIMEN_POSITION);
        wrist.queue_position(ArmConfig.WRIST_SPECIMEN_POSITION);
        arm_current = "SPEC";
    }

    public void park_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_PARK_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_PARK_POSITION);
        wrist.queue_position(ArmConfig.WRIST_PARK_POSITION);
        arm_current = "PARK";
    }

    public void open_claw() {
        claw.queue_position(ArmConfig.CLAW_OPEN_POSITION);
        claw_state = false;
    }

    public void close_claw() {
        claw.queue_position(ArmConfig.CLAW_CLOSE_POSITION);
        claw_state = true;
    }
}