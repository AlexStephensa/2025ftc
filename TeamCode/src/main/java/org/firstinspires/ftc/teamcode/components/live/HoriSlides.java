package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

import static org.firstinspires.ftc.teamcode.components.live.HoriSlides.*;

//@Config

class HoriSlideConfig {
    public static int UNIT_LENGTH = 240; //In encoder counts
    public static int MAX_LENGTH = 12;
    public static int MIN_LENGTH = 0;
    /*
    public static double PID_P = 15;
    public static double PID_I = 0.1;
    public static double PID_D = 4;
    */

}

public class HoriSlides {

    //// SERVOS ////
    public ServoQUS horiServL;
    public ServoQUS horiServR;

    //// SENSORS ////
    public DigitalChannel limit_switchh;

    public int Length;

    {
        name = "HoriSlides";
    }

    public HoriSlides(Robot robot)
    {
        super(robot);
    }

    @Override
    public void registerHardware (HardwareMap hwmap)
    {
        super.registerHardware(hwmap);

        //// SERVOS ////
        horiServL     = new ServoQUS(hwmap.get(Servo.class, "horiServL"));
        horiServR     = new ServoQUS(hwmap.get(Servo.class, "horiServR"));

        limit_switchh = hwmap.get(DigitalChannel.class, "limit_switchv");
    }
}