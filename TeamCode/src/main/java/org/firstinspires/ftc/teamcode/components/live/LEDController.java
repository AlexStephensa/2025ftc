package org.firstinspires.ftc.teamcode.components.live;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;

import static org.firstinspires.ftc.teamcode.components.live.LEDControllerConfig.ALLIANCE_COLOR;
import static org.firstinspires.ftc.teamcode.components.live.LEDControllerConfig.BLUE;
import static org.firstinspires.ftc.teamcode.components.live.LEDControllerConfig.RED;
import static org.firstinspires.ftc.teamcode.components.live.LEDControllerConfig.FLASH_RATE;
import static org.firstinspires.ftc.teamcode.components.live.LEDControllerConfig.FLASH_COUNT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Config
class LEDControllerConfig {
    static public BlinkinPattern BLUE = COLOR_WAVES_OCEAN_PALETTE;
    static public BlinkinPattern RED = COLOR_WAVES_LAVA_PALETTE;
    static public BlinkinPattern ALLIANCE_COLOR = BLUE;
    static public double FLASH_RATE = 0.5; // S
    static public int FLASH_COUNT = 6;

}

public class LEDController extends Component {

    private RevBlinkinLedDriver led_control;

    private Robot robot;


    BlinkinPattern last_pattern;
    BlinkinPattern current_pattern = LEDControllerConfig.ALLIANCE_COLOR;

    // Flash Control
    boolean flash = false;
    boolean blink = false;
    int count;
    int flash_count;
    double flash_rate;
    double flash_start;


    {
        name = "LED";
    }

    public LEDController(Robot robot) {
        super(robot);

        this.robot = robot;
    }

    @Override
    public void registerHardware (HardwareMap hwmap) {
        led_control = hwmap.get(RevBlinkinLedDriver.class, "led_control");
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        BlinkinPattern new_pattern = current_pattern;

        if (flash) {
            if (count < LEDControllerConfig.FLASH_COUNT) {
                if (0 < ((robot.opmode.getRuntime() - flash_start) - LEDControllerConfig.FLASH_RATE)) {
                    flash_start = robot.opmode.getRuntime();

                    new_pattern = blink ? current_pattern : BlinkinPattern.BLACK;

                    count += blink ? 1 : 0;
                    blink = !blink;
                }

            } else {
                stop_flash();
            }
        } else {
            new_pattern = current_pattern;
        }
        if (new_pattern != last_pattern) {
            update_pattern(new_pattern);
        }

        last_pattern = new_pattern;
    }

    @Override
    public void startup() {
        super.startup();

        update_pattern(COLOR_WAVES_OCEAN_PALETTE);

    }

    @Override
    public void shutdown() {
        super.shutdown();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("PATTERN CURRENT", current_pattern);
        telemetry.addData("BLINK", blink);
    }

    public void update_pattern(BlinkinPattern pattern) {
        led_control.setPattern(pattern);
    }

    public void set_pattern(BlinkinPattern pattern) {
        current_pattern = pattern;
    }

    public void pattern_flash(BlinkinPattern pattern) {
        pattern_flash(pattern, FLASH_COUNT, FLASH_RATE);
    }

    public void pattern_flash(BlinkinPattern pattern, int count, double rate) {
        current_pattern = pattern;
        flash_start = robot.opmode.getRuntime();
        flash = true;
        flash_count = count;
        flash_rate = rate;
    }

    public void stop_flash() {
        flash = false;
        blink = false;
        count = 0;
        current_pattern = LEDControllerConfig.ALLIANCE_COLOR;
    }

    public void blue_alliance() {
        ALLIANCE_COLOR = BLUE;
        set_pattern(ALLIANCE_COLOR);
    }
    public void red_alliance() {
        ALLIANCE_COLOR = RED;
        set_pattern(ALLIANCE_COLOR);
    }
}