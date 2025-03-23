package org.firstinspires.ftc.teamcode.opmodes.test;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robots.LiveRobot;

public abstract class TestTeleopBase extends LinearOpMode {

    protected LiveRobot robot;

    private ArrayMap<Double, Runnable> todo_tasks = new ArrayMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LiveRobot(this);

        RobotLog.dd("---", "robot.startup");
        robot.startup();

        RobotLog.dd("---", "on_init");
        on_init();

        RobotLog.dd("---", "waitForStart");
        waitForStart();
        RobotLog.dd("---", "on_start");
        on_start();

        while(opModeIsActive() && !isStopRequested()) {

            on_loop();

            for(int entry = 0; entry < todo_tasks.size(); entry++) {
                if(todo_tasks.keyAt(entry) <= getRuntime()) {
                    todo_tasks.valueAt(entry).run();
                    todo_tasks.remove(todo_tasks.keyAt(entry));
                }
            }

        }
        RobotLog.dd("---", "on_stop");
        on_stop();

        RobotLog.dd("---", "robot.shutdown");
        robot.shutdown();

        RobotLog.dd("---", "stop");
        stop();

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
     * Called repeatedly while the program is running
     */
    public abstract void on_loop();

    /**
     * Waits until @seconds passes then runs @command
     * @param command runnable command
     * @param seconds seconds to wait
     */
    public void run_in(Runnable command, double seconds) {
        todo_tasks.put(getRuntime() + (seconds), command);
    }

    /**
     * Waits until @milliseconds passes then runs @command
     * @param command runnable command
     * @param milliseconds milliseconds to wait
     */
    public void run_in(Runnable command, int milliseconds) {
        todo_tasks.put(getRuntime() + ((milliseconds)/1000.0), command);
    }
}
