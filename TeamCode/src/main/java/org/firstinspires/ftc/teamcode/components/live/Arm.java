package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class ArmConfig {
    static public double ELBOW_L_TRANSFER_POSITION = 0;
    static public double ELBOW_L_WAITING_POSITION = 0;
    static public double ELBOW_L_SPECIMEN_POSITION = 0;
    static public double ELBOW_L_BASKET_POSITION = 0;

    static public double ELBOW_R_TRANSFER_POSITION = 0;
    static public double ELBOW_R_WAITING_POSITION = 0;
    static public double ELBOW_R_SPECIMEN_POSITION = 0;
    static public double ELBOW_R_BASKET_POSITION = 0;

    static public double WRIST_TRANSFER_POSITION = 0;
    static public double WRIST_SPECIMEN_POSITION = 0;
    static public double WRIST_BASKET_POSITION = 0;

    static public double CLAW_CLOSE_POSITION = 0;
    static public double CLAW_OPEN_POSITION = 0;
}

public class Arm extends Component {
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
        wrist     = new ServoQUS(hwmap.get(Servo.class, "wrist"));
        claw     = new ServoQUS(hwmap.get(Servo.class, "claw"));
    }

    @Override
    public void startup() {
        transfer_position();
        close_claw();
    }

    @Override
    public void shutdown() {
        //shut down
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    public void transfer_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_TRANSFER_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_TRANSFER_POSITION);
        wrist.queue_position(ArmConfig.WRIST_TRANSFER_POSITION);
    }

    public void waiting_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_WAITING_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_WAITING_POSITION);
        wrist.queue_position(ArmConfig.WRIST_TRANSFER_POSITION);
        open_claw();
    }

    public void basket_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_BASKET_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_BASKET_POSITION);
        wrist.queue_position(ArmConfig.WRIST_BASKET_POSITION);
    }

    public void specimen_position() {
        elbow_l.queue_position(ArmConfig.ELBOW_L_SPECIMEN_POSITION);
        elbow_r.queue_position(ArmConfig.ELBOW_R_SPECIMEN_POSITION);
        wrist.queue_position(ArmConfig.WRIST_SPECIMEN_POSITION);
    }

    public void open_claw() {
        claw.queue_position(ArmConfig.CLAW_OPEN_POSITION);
    }

    public void close_claw() {
        claw.queue_position(ArmConfig.CLAW_CLOSE_POSITION);
    }
}