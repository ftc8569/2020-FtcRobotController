package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.StaticVaribaleTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled

@TeleOp(group = "ET")
public class StaticVariableTele extends OpMode {
    public static int number = 1;
    public void init() {
        telemetry.addData(String.valueOf(number), number);
        System.out.println(number);
    }

    public void loop() {


    }

    public void stop() {


    }
}
