package org.firstinspires.ftc.teamcode.opmodes.test;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robots.TestRobot;

public abstract class TestAutoBase extends LinearOpMode {

    protected TestRobot robot;

    private ArrayMap<Double, Runnable> todo_tasks = new ArrayMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TestRobot(this);

        // Start up the robot as soon as the program is initialized
        RobotLog.dd("---", "robot.startup");
        robot.startup();

        RobotLog.dd("---", "on_init");
        on_init();
        RobotLog.dd("---", "waitForStart");
        waitForStart();
        RobotLog.dd("---", "on_start");
        on_start();

        RobotLog.dd("---", "on_stop");
        on_stop();

        // Shut the robot down as soon as the program is finished
        RobotLog.dd("---", "robot.shutdown");
        robot.shutdown();

        //RobotLog.dd("---", "stop");
        //stop();

        RobotLog.dd("---", "end");
    }

    /**
     * Called when init is pressed, runs once
     */
    public abstract void on_init();

    /**
     * Called when start is pressed, runs once
     */
    public abstract void on_start();

    /**
     * Called when stop is pressed, runs once
     */
    public abstract void on_stop();

    /**
     * Pauses the robot until @seconds passes
     * @param seconds seconds to wait
     */
    protected void halt(double seconds) {
        double start = robot.opmode.getRuntime();
        while ((robot.opmode.getRuntime() - start) < seconds && robot.opmode.opModeIsActive());
    }

    /**
     * Waits until @seconds passes then runs @command
     * @param command runnable command
     * @param seconds seconds to wait
     */
    public void run_in(Runnable command, double seconds) {
        todo_tasks.put(getRuntime() + (seconds), command);
    }
}
