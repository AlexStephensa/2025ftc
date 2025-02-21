package org.firstinspires.ftc.teamcode.components.live;


import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.constants.IntakeConst;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

@Config
class IntakeConfig {
    // Servo Positions for Sample Transfer
    public static double PITCH_L_TRANSFER_POSITION = 0.4;
    public static double PITCH_R_TRANSFER_POSITION = 0.4;

    // Servo Positions for Intake from Field
    public static double PITCH_L_INTAKE_POSITION = 0.57;
    public static double PITCH_R_INTAKE_POSITION = 0.57;

    public static double PITCH_INTAKE_TWEAK = 0.03;

    public static int COLOR_UPDATE_RATE = 5; // Color Sensor update interval
    public static double COLOR_CUTOFF = 0.5; // Cut off value of color sensor

    public static int SPIT_DURATION = 300_000_000; // nanoseconds, 0.3s
}

public class Intake extends Component {
    private NormalizedRGBA current_color_RGBA;
    public int current_color = IntakeConst.SAMPLE_NONE;

    public int intake_color_wanted = IntakeConst.SAMPLE_BLUE;

    private int intake_angle;

    private long spitting_since = -1;

    public boolean auto_run = false;

    //// SERVOS ////
    private ServoQUS pitch_l;
    private ServoQUS pitch_r;
    private CRServoQUS intake;

    //// SENSORS ////
    private RevColorSensorV3 color_sensor;

    {
        name = "Intake";
    }
    
    public Intake(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// SERVOS ////
        pitch_l = new ServoQUS(hwmap.get(Servo.class, "pitchL"));
        pitch_r = new ServoQUS(hwmap.get(Servo.class, "pitchR"));
        pitch_r.servo.setDirection(Servo.Direction.REVERSE);

        intake = new CRServoQUS(hwmap.get(CRServo.class, "intake"));
        intake.servo.setDirection(DcMotorSimple.Direction.REVERSE);

        //// SENSORS ////
        color_sensor = hwmap.get(RevColorSensorV3.class, "intakeColor");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        if (auto_run) intake_run_auto(1);

        pitch_l.update();
        pitch_r.update();
        intake.update();

        if (robot.cycle % IntakeConfig.COLOR_UPDATE_RATE == 0) {
            current_color_RGBA = color_sensor.getNormalizedColors();
            intake_color_check();
        }
    }

    @Override
    public void startup() {
        super.startup();
        intake_transfer();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("SPINNER",TELEMETRY_DECIMAL.format(intake.servo.getPower()));
        telemetry.addData("INTAKE CURRENT", intake_angle());
        telemetry.addData("COLOR RGB",current_color_RGBA.red + " " + current_color_RGBA.green + " " + current_color_RGBA.blue);
        telemetry.addData("COLOR", current_color);
    }

    public void intake_run(double speed, Gamepad gamepad1, Gamepad gamepad2) {
        if (speed > 0) {
            if (intake_angle == IntakeConst.INTAKE) {
                pitch_l.queue_position(IntakeConfig.PITCH_L_INTAKE_POSITION + IntakeConfig.PITCH_INTAKE_TWEAK);
                pitch_r.queue_position(IntakeConfig.PITCH_R_INTAKE_POSITION + IntakeConfig.PITCH_INTAKE_TWEAK);
            }

            gamepad2.setLedColor(
                    (current_color == IntakeConst.SAMPLE_RED || current_color == IntakeConst.SAMPLE_YELLOW) ? 1 : 0,
                    (current_color == IntakeConst.SAMPLE_YELLOW) ? 1 : 0,
                    (current_color == IntakeConst.SAMPLE_BLUE) ? 1 : 0,
                    LED_DURATION_CONTINUOUS
            );

            if (!(current_color == IntakeConst.SAMPLE_NONE) && !(current_color == intake_color_wanted) && !(current_color == IntakeConst.SAMPLE_YELLOW)) {
                if (spitting_since == -1) {
                    spitting_since = System.nanoTime();
                } else if ((System.nanoTime() - spitting_since) < IntakeConfig.SPIT_DURATION) {
                    intake.queue_power(-1);
                }
            } else {
                if ((current_color == intake_color_wanted) || (current_color == IntakeConst.SAMPLE_YELLOW)) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    intake_transfer();
                }
                intake.queue_power(speed);
                spitting_since = -1;
            }
        } else {
            intake.queue_power(speed);
            if (intake_angle == IntakeConst.INTAKE) {
                pitch_l.queue_position(IntakeConfig.PITCH_L_INTAKE_POSITION);
                pitch_r.queue_position(IntakeConfig.PITCH_R_INTAKE_POSITION);
            }
        }
    }

    public void intake_run_auto(double speed) {
        if (current_color == IntakeConst.SAMPLE_NONE) {
            pitch_l.queue_position(IntakeConfig.PITCH_L_INTAKE_POSITION + IntakeConfig.PITCH_INTAKE_TWEAK);
            pitch_r.queue_position(IntakeConfig.PITCH_R_INTAKE_POSITION + IntakeConfig.PITCH_INTAKE_TWEAK);
        } else {
            intake_transfer();
        }
        intake.queue_power(speed);
    }

    public void intake_color_check() {
        if (current_color_RGBA.alpha < IntakeConfig.COLOR_CUTOFF) {
            current_color = IntakeConst.SAMPLE_NONE;
        } else if (current_color_RGBA.red > current_color_RGBA.blue && current_color_RGBA.red > current_color_RGBA.green) {
            current_color = IntakeConst.SAMPLE_RED;
        } else if (current_color_RGBA.blue > current_color_RGBA.red && current_color_RGBA.blue > current_color_RGBA.green) {
            current_color = IntakeConst.SAMPLE_BLUE;
        } else if (current_color_RGBA.green > current_color_RGBA.red && current_color_RGBA.green > current_color_RGBA.blue) {
            current_color = IntakeConst.SAMPLE_YELLOW;
        }
    }

    public void intake_transfer() {
        pitch_l.queue_position(IntakeConfig.PITCH_L_TRANSFER_POSITION);
        pitch_r.queue_position(IntakeConfig.PITCH_R_TRANSFER_POSITION);
        intake_angle = IntakeConst.TRANS;
    }

    public void intake_intake() {
        pitch_l.queue_position(IntakeConfig.PITCH_L_INTAKE_POSITION);
        pitch_r.queue_position(IntakeConfig.PITCH_R_INTAKE_POSITION);
        intake_angle = IntakeConst.INTAKE;
    }

    public void toggle_wanted_color() {
        intake_color_wanted = (intake_color_wanted == IntakeConst.SAMPLE_BLUE) ? IntakeConst.SAMPLE_RED : IntakeConst.SAMPLE_BLUE;
    }

    public void setAuto_run(boolean run) {
        auto_run = run;
    }

    private String intake_angle() {
        return (intake_angle == IntakeConst.INTAKE) ? "INTAKE" : "TRANS";
    }

}
