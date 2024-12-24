package org.firstinspires.ftc.teamcode.components.live;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class IntakeConfig {
    // Servo Positions for Initialization
    public static double pitchL_init = 0.5;
    public static double pitchR_init = 0.5;

    // Servo Positions for Sample Transfer
    public static double pitchL_tran = 0.4;
    public static double pitchR_tran = 0.4;

    // Servo Positions for Reach Movement
    public static double pitchL_cradel = 0.45;
    public static double pitchR_cradel = 0.45;

    // Servo Positions for Intake from Field
    public static double pitchL_intake = 0.57;
    public static double pitchR_intake = 0.57;

    public static String intakeCurrent = "Initialization";


    public static int COLOR_UPDATE_RATE = 5; // Color Sensor update interval
    public static int COLOR_CUTOFF = 300; // Cut off value of color sensor
    public static String current_color;
}

public class Intake extends Component {

    //// SERVOS ////
    public ServoQUS pitchL;
    public ServoQUS pitchR;
    public CRServoQUS intake;

    //// SENSORS ////
    public RevColorSensorV3 intakeColor;

    public int ColorRate = IntakeConfig.COLOR_UPDATE_RATE;

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
        intakeColor = hwmap.get(RevColorSensorV3.class, "intakeColor");
        intakeColor.enableLed(false);



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

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("SPINNER",TELEMETRY_DECIMAL.format(intake.servo.getPower()));
        telemetry.addData("INTAKE CURRENT",IntakeConfig.intakeCurrent);
        telemetry.addData("COLOR RGBA",intakeColor.red() + " " + intakeColor.blue() + " " + intakeColor.green() + " " + intakeColor.alpha());
        telemetry.addData("COLOR", IntakeConfig.current_color);
    }

    public void intakeRun(double speed) {
        intake.queue_power(speed);
    }
    
    public void update_intake() {
        pitchL.update();
        pitchR.update();
        intake.update();
    }

    public void intake_colorCheck() {
        if (intakeColor.alpha() < IntakeConfig.COLOR_CUTOFF) {
            IntakeConfig.current_color = "none";
        } else if (intakeColor.red() > intakeColor.blue() && intakeColor.red() > intakeColor.green()) {
            IntakeConfig.current_color = "red";
        } else if (intakeColor.blue() > intakeColor.red() && intakeColor.blue() > intakeColor.green()) {
            IntakeConfig.current_color = "blue";
        } else if (intakeColor.green() > intakeColor.red() && intakeColor.green() > intakeColor.red()) {
            IntakeConfig.current_color = "yellow";
        }

    }

    public void intake_init() {
        pitchL.queue_position(IntakeConfig.pitchL_init);
        pitchR.queue_position(IntakeConfig.pitchR_init);
        IntakeConfig.intakeCurrent = "INITIAL";
    }

    public void intake_cradle() {
        pitchL.queue_position(IntakeConfig.pitchL_cradel);
        pitchR.queue_position(IntakeConfig.pitchR_cradel);
        IntakeConfig.intakeCurrent = "CRADLE";
    }

    public void intake_transfer() {
        pitchL.queue_position(IntakeConfig.pitchL_tran);
        pitchR.queue_position(IntakeConfig.pitchR_tran);
        IntakeConfig.intakeCurrent = "TRANS";
    }

    public void intake_intake() {
        pitchL.queue_position(IntakeConfig.pitchL_intake);
        pitchR.queue_position(IntakeConfig.pitchR_intake);
        IntakeConfig.intakeCurrent = "INTAKE";
    }
}
