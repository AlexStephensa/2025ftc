package org.firstinspires.ftc.teamcode.robots;

import static org.firstinspires.ftc.teamcode.robots.RobotConfig.BULK_READ_1_CYCLE;
import static org.firstinspires.ftc.teamcode.robots.RobotConfig.BULK_READ_2_CYCLE;
import static org.firstinspires.ftc.teamcode.robots.RobotConfig.COMPONENT_UPDATE_CYCLE;
import static org.firstinspires.ftc.teamcode.robots.RobotConfig.FREQ_CHECK_CYCLE;
import static org.firstinspires.ftc.teamcode.robots.RobotConfig.TELEMETRY_CYCLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.components.Component;

import java.util.ArrayList;
import java.util.List;

@Config
class RobotConfig {
    // Interval in cycles at which we call update on all the components
    public static int COMPONENT_UPDATE_CYCLE = 1;
    // Interval in cycles at which we bulk read from Rev Hub 1
    public static int BULK_READ_1_CYCLE = 1;
    // Interval in cycles at which we bulk read from Rev Hub 2
    public static int BULK_READ_2_CYCLE = 5;
    // Interval in cycles at which we send telemetry to the phone
    public static int TELEMETRY_CYCLE = 40;
    // Interval in cycles at which we calculate cycle frequency
    public static int FREQ_CHECK_CYCLE = 40;
}

public class Robot {

    HardwareMap hwmap;
    public LinearOpMode opmode;

    ArrayList<Component> components = new ArrayList<>();

    public String name;

    public ArrayList<String> warnings = new ArrayList<>();

    boolean running = false;

    Telemetry telemetry;

    protected long last_update = System.nanoTime();
    protected int update_freq = 0;

    public int cycle = 0;

    Runnable update_thread;

    List<LynxModule> expansion_hubs;

    // The Update Thread
    // Should be called as fast as possible. Does all reads and writes to the rev hub
    class UpdateThread implements Runnable {

        Robot robot;

        public UpdateThread(Robot robot) {
            super();
            this.robot = robot;
        }

        @Override
        public void run() {
            while(robot.running) {
                robot.update();
            }
        }
    }

    public Robot(LinearOpMode opmode) { // FIRST THIS IS *YOUR* FAULT IT HAS TO BE THIS WAY AND I HATE IT AND YOU SHOULD BE ASHAMED
        this.opmode = opmode;
        this.telemetry = this.opmode.telemetry;

        this.hwmap  = opmode.hardwareMap;
        registerHardware(this.hwmap);

        this.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE); // Begone jumpy telemetry
    }

    public void startup() {
        running = true;

        for (LynxModule hub : expansion_hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (Component component : components) {
            component.startup();
        }

        update_thread = new UpdateThread(this);

        new Thread(update_thread).start();

        // Check for good battery Voltage
        if (expansion_hubs.get(0).getInputVoltage(VoltageUnit.VOLTS) <= 12.25 || expansion_hubs.get(1).getInputVoltage(VoltageUnit.VOLTS) <= 12.25) {
            addWarning("Robot Voltage Low");
        }
    }

    public void shutdown() {
        running = false;

        for (Component component : components) {
            component.shutdown();
        }
    }

    /**
     * This method is called as fast as possible by the update thread
     */
    public void update() {

        if(cycle % BULK_READ_1_CYCLE == 0) {
            expansion_hubs.get(0).clearBulkCache();
        }

        // Bulk read from rev hub 2
        if(cycle % BULK_READ_2_CYCLE == 0) {
            expansion_hubs.get(1).clearBulkCache();
        }

        // Call update on every single component
        if(cycle % COMPONENT_UPDATE_CYCLE == 0) {
            for (Component component : components) {
                component.update(opmode);
            }
        }

        // Update telemetry on the dashboard and on the phones
        if (cycle % TELEMETRY_CYCLE == 0) {

            updateTelemetry();

            for (Component component : components) {
                component.updateTelemetry(telemetry);
            }

            telemetry.update();
        }

        // Recalculate our update thread frequency
        if (cycle % FREQ_CHECK_CYCLE == 0) {
            long update_duration = System.nanoTime()-last_update;
            update_freq = ((update_duration/(double)1000000000) * FREQ_CHECK_CYCLE) != 0 ? (int)((1/(update_duration/(double)1000000000)) * FREQ_CHECK_CYCLE) : Integer.MAX_VALUE;

            last_update = System.nanoTime();
        }

        // This robot cycle is complete, increment our cycle counter by one
        cycle++;
    }

    /**
     * For updating the telemetry on the phones
     */
    public void updateTelemetry() {

        if (warnings.size() > 0) {
            telemetry.addData("[WARNINGS]","");
            for (int i = 0; i < warnings.size(); i++) {
                telemetry.addData(String.valueOf(i+1), warnings.get(i));
            }
        }

        telemetry.addData("[RBT "+name+"]", components.size()+" components");
        telemetry.addData("FREQ", update_freq);
    }

    /**
     * This should automatically be called whenever you make a new component attached to a robot instance
     * Basically just adds the component to a list of registered components, and attaches all hardware the component needs from the configuration
     */
    public void registerComponent(Component component) {
        component.registerHardware(hwmap);
        components.add(component);
    }

    /**
     * Called on robot startup, registers all hardware the robot instance needs to use, in this case both the rev hubs
     */
    public void registerHardware(HardwareMap hwmap) {
        expansion_hubs = hwmap.getAll(LynxModule.class);
    }

    /**
     * Add a warning to be displayed on the phone for when something is amiss and the robot should not be run
     */
    public void addWarning(String warning) {
        warnings.add(warning);
    }
}
