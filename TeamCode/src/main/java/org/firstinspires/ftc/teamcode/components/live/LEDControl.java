package org.firstinspires.ftc.teamcode.components.live;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.constants.LEDConst;
import org.firstinspires.ftc.teamcode.robots.Robot;

@Config
class LEDConfig {
    public static BlinkinPattern ALLIANCE_COLOR = LEDConst.BLUE;
    public static double FLASH_RATE = 500; // MS
    public static int FLASH_COUNT = 6;
    public static int NANO_PER_MILLI = 1000000;
    public static BlinkinPattern setAllianceColor(BlinkinPattern pattern) {
        ALLIANCE_COLOR = pattern;
        return ALLIANCE_COLOR;
    }
}

public class LEDControl extends Component {
    BlinkinPattern current_pattern = LEDConfig.ALLIANCE_COLOR;
    BlinkinPattern last_pattern = LEDConst.DEFAULT;

    // Flash Control
    BlinkinPattern flash_pattern;
    boolean blink = false;
    int count = 0;
    int flash_count = 0;
    double flash_rate = LEDConfig.FLASH_RATE;
    long flash_start = 0;

    private RevBlinkinLedDriver led_control;

    {
        name = "LED_Controller";
    }

    public LEDControl(Robot robot) { super(robot); }

    @Override
    public void registerHardware (HardwareMap hwmap) {
        led_control = hwmap.get(RevBlinkinLedDriver.class, "led_control");
        led_control.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void update(OpMode opmode) {
        super.update(opmode);

        if (count < flash_count) {
            if (0 < ((double) (System.nanoTime() - flash_start) / LEDConfig.NANO_PER_MILLI - LEDConfig.FLASH_RATE)) {
                flash_start = System.nanoTime();
                set_pattern(blink ? flash_pattern : LEDConst.OFF);

                count += blink ? 0 : 1;
                blink = !blink;
            }
        } else {
            blink = false;
            set_pattern(LEDConfig.ALLIANCE_COLOR);
        }
        if (this.current_pattern != this.last_pattern) {
            update_pattern(this.current_pattern);
        }
        this.last_pattern = this.current_pattern;
    }

    @Override
    public void startup() {
        super.startup();
        led_control.setPattern(LEDConst.BLUE);
    }

    @Override
    public void shutdown() {
        super.shutdown();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        telemetry.addData("CURRENT PATTERN", pattern_name(current_pattern));
    }

    public void update_pattern(BlinkinPattern pattern) {
        led_control.setPattern(pattern);
    }

    public void set_pattern(BlinkinPattern pattern) {
        current_pattern = pattern;
    }

    public void pattern_flash(BlinkinPattern pattern) {
        pattern_flash(pattern, LEDConfig.FLASH_COUNT, LEDConfig.FLASH_RATE);
    }

    public void pattern_flash(BlinkinPattern pattern, int count, double rate) {
        this.flash_pattern = pattern;
        this.flash_count = count;
        this.flash_rate = rate / 1000;
        this.count = 0;
    }

    public void toggle_alliance_color() {
        if (LEDConfig.ALLIANCE_COLOR == LEDConst.BLUE) {
            red_alliance();
        } else {
            blue_alliance();
        }
    }
    public void blue_alliance() {
        set_pattern(LEDConfig.setAllianceColor(LEDConst.BLUE));
    }
    public void red_alliance() {
        set_pattern(LEDConfig.setAllianceColor(LEDConst.RED));
    }

    private String pattern_name(BlinkinPattern pattern) {
        if (pattern.equals(LEDConst.BLUE)) {
            return "BLUE";
        }
        if (pattern.equals(LEDConst.RED)) {
            return "RED";
        }
        if (pattern.equals(LEDConst.YELLOW)) {
            return "YELLOW";
        }
        if (pattern.equals(LEDConst.OFF)) {
            return "OFF";
        }
        return "DEFAULT";
    }
}