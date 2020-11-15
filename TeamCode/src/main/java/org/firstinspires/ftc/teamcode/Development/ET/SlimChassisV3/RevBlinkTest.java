package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Dev: RevBlinkTest", group = "Development")
@Disabled
public class RevBlinkTest extends OpMode {
    RevBlinkinLedDriver led;
    public static RevBlinkinLedDriver.BlinkinPattern blink = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    @Override
    public void init() {
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    @Override
    public void loop() {
        led.setPattern(blink);
    }
}
