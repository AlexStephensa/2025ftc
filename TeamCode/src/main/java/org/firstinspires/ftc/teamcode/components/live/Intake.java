/*package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

//@Config
class IntakeConfig {
    public static double DROPPER_CLOSED = 0.75f;
    public static double DROPPER_OPEN = 1.0;
}

public class Intake extends Component {

    public DcMotorQUS intake;
    public ServoQUS dropper; //positional Servo

    {
        name = "Intake";
    }

    public Intake(Robot robot) {
        super(robot);
    }

    public void drop() {
        dropper.queue_position(IntakeConfig.DROPPER_OPEN);
    }

    public void undrop() {
        dropper.queue_position(IntakeConfig.DROPPER_CLOSED);
    }

    public void spin(double speed) {
        intake.queue_power(speed);
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        dropper.update();
        intake.update();
    }

    @Override
    public void startup() {
        super.startup();

        undrop();
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        intake = new DcMotorQUS(hwmap.get(DcMotorEx.class, "intake"));
        dropper = new ServoQUS(hwmap.get(Servo.class, "dropper"));
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        telemetry.addData("SPEED", TELEMETRY_DECIMAL.format(intake.motor.getPower()));
    }
}
*/
