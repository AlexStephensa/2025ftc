/*package org.firstinspires.ftc.teamcode.components.live;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
class LinearLiftConfig {



}

public class LinearLift extends Component {

    //Motors//
    public DcMotorEx lift_l;
    public DcMotorEx lift_r;

    //Sensors//
    public DigitalChannel limit_switchV;

    {
        name = "LinearLift";
    }

    public LinearLift(Robot robot){
        super(robot);
    }

    public void lift_up() {
        lift_l.setPower(-1);
        lift_r.setPower(1);
    }

    public void lift_power_zero() {

        lift_l.setPower(0);
        lift_r.setPower(0);
    }

    public void lift_down() {
        lift_l.setPower(1);
        lift_r.setPower(-1);
       if (limit_switchV.getState()){
           lift_l.setPower(0);
           lift_r.setPower(0);
       }
    }

    @Override
    public void startup(){
        super.startup();

    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        lift_l     = hwmap.get(DcMotorEx.class, "lift_l");
        lift_r     = hwmap.get(DcMotorEx.class, "lift_r");

    }
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        //telemetry.addData("LL TURNS",TELEMETRY_DECIMAL.format(lift_l.getCurrentPosition()));
        //telemetry.addData("RL TURNS",TELEMETRY_DECIMAL.format(lift_r.getCurrentPosition()));

        //telemetry.addData("LIFT BUSY",lift_l.isBusy()+" "+lift_r.isBusy());

        //telemetry.addData("LIM", !limit_switchV.getState());
    }
}*/