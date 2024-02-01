package org.firstinspires.ftc.teamcode.components.live;

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
class CrossbowConfig {
    public static double CROSSBOW_UNSHOT = 0.84f;
    public static double CROSSBOW_SHOT = 1f;
}

public class Crossbow extends Component {

    public ServoQUS crossbow; //positional Servo

    {
        name = "Crossbow";
    }

    public Crossbow(Robot robot) {
        super(robot);
    }

    public void shoot() {
        crossbow.queue_position(CrossbowConfig.CROSSBOW_SHOT);
    }

    public void unshoot() {
        crossbow.queue_position(CrossbowConfig.CROSSBOW_UNSHOT);
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        crossbow.update();
    }

    @Override
    public void startup() {
        super.startup();

        unshoot();
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        crossbow = new ServoQUS(hwmap.get(Servo.class, "crossbow"));
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }
}
