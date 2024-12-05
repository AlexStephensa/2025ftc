package org.firstinspires.ftc.teamcode.components.live;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class IntakeConfig {
    // Servo Positions for Initialization
    public static double pitchL_init = 0;
    public static double pitchR_init = 0;

    // Servo Positions for Sample Transfer
    public static double pitchL_tran = 0;
    public static double pitchR_tran = 0;

    // Servo Positions for Reach Movement
    public static double pitchL_cradel = 0;
    public static double pitchR_cradel = 0.57;

    // Servo Positions for Intake from Field
    public static double pitchL_intake = 0;
    public static double pitchR_intake = 0;


    public static int COLOR_UPDATE = 5; // Color Sensor update interval
    public static int COLOR_DISTANCE = 75; // Cut off distance of color sensor in mm
}

public class Intake extends Component {

    //// SERVOS ////
    public ServoQUS pitchL;
    public ServoQUS pitchR;
    public CRServoQUS intake;

    //// SENSORS ////
    public ColorRangeSensor intakeColor;
    public boolean color_sensor_enabled = false;
    public double last_distance = 0;
public String current_color = null;

    {
        name = "Intake";
    }
    
    public Intake(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// SERVOS ////
        pitchL = new ServoQUS(hwmap.get(Servo.class, "pitchL"));
        pitchR = new ServoQUS(hwmap.get(Servo.class, "pitchR"));
        pitchR.servo.setDirection(Servo.Direction.REVERSE);

        intake = new CRServoQUS(hwmap.get(CRServo.class, "intake"));
        intake.servo.setDirection(DcMotorSimple.Direction.REVERSE);

        //// SENSORS ////
        intakeColor = hwmap.get(ColorRangeSensor.class, "intakeColor");
        public static String current_color = null;

    }

    //@Override
    public void update(OpMode opmode) {
        super.update(opmode);

        update_intake();
    }

    @Override
    public void startup() {
        super.startup();
        intakeRun(0);
        intake_init();
    }

    public void intakeRun(double speed) {
        intake.queue_power(speed);
    }
    
    public void update_intake() {
        pitchL.update();
        pitchR.update();
        intake.update();
    }

    public void intake_colorCheck(String color) {
        current_color = intakeColor.getRawColor();
    }

    public void intake_init() {
        pitchL.queue_position(IntakeConfig.pitchL_init);
        pitchR.queue_position(IntakeConfig.pitchR_init);
    }

    public void intake_cradel() {
        pitchL.queue_position(IntakeConfig.pitchL_cradel);
        pitchR.queue_position(IntakeConfig.pitchR_cradel);
    }

    public void intake_transfer() {
        pitchL.queue_position(IntakeConfig.pitchL_tran);
        pitchR.queue_position(IntakeConfig.pitchR_tran);
    }

    public void intake_intake() {
        pitchL.queue_position(IntakeConfig.pitchL_intake);
        pitchR.queue_position(IntakeConfig.pitchR_intake);
    }
}
