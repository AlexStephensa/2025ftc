package org.firstinspires.ftc.teamcode.components.live;

//import com.acmerobotics.dashboard.config.Config;

//import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.UNIT_LENGTH;
//import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MAX_LENGTH;
//import static org.firstinspires.ftc.teamcode.components.live.LiftConfig.MIN_LENGTH;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;

//@Config

class ReachConfig {
    public static int UNIT_LENGTH = 240; //In encoder counts
    public static int MAX_LENGTH = 12;
    public static int MIN_LENGTH = 0;
    /*
    public static double PID_P = 15;
    public static double PID_I = 0.1;
    public static double PID_D = 4;
    */

}

public class Reach {
    //// SERVOS ////
    public ServoQUS horiServL;
    public ServoQUS horiServR;

    //// SENSORS ////
    public DigitalChannel limit_switchH;

    public double Length;
    public double MAX_REACH; //Double distance in inch that reach can extend to
    public double MIN_REACH; //Double distance in inch that reach can contract to

    protected String name = "Reach";

    public Reach(Robot robot)
    {
        super();
    }

    //@Override
    public void registerHardware (HardwareMap hwmap) {
        //// SERVOS ////
        horiServL     = new ServoQUS(hwmap.get(Servo.class, "horiServL"));
        horiServR     = new ServoQUS(hwmap.get(Servo.class, "horiServR"));

        limit_switchH = hwmap.get(DigitalChannel.class, "limit_switchV");
    }
}