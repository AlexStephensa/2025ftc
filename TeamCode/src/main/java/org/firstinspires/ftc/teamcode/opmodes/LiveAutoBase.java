package org.firstinspires.ftc.teamcode.opmodes;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.LiveRobot;

public abstract class LiveAutoBase extends LinearOpMode {

    protected LiveRobot robot;

    private ArrayMap<Double, Runnable> todo_tasks = new ArrayMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LiveRobot(this);
        // Start up the robot as soon as the program is initialized
        robot.startup();
        on_init();
        waitForStart();
        on_start();
        on_stop();
        // Shut the robot down as soon as the program is finished
        robot.shutdown();
        stop();
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
        resetRuntime();
        while (getRuntime() < seconds && opModeIsActive()) {}
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
