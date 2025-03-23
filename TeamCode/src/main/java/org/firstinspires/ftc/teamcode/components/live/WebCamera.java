package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Config
class WebCameraConfig {

}

public class WebCamera extends Component {


    {
        name = "Web Camera";
    }

    public WebCamera(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap) {
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);
    }

    @Override
    public void startup() {
        super.startup();
    }

    @Override
    public void shutdown() {
        super.shutdown();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }
}