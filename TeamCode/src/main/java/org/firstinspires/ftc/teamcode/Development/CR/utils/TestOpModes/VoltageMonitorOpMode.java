package org.firstinspires.ftc.teamcode.Development.CR.utils.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "CR-VoltageMonitorOpMode", group = "CR-VoltageMonitor")
public class VoltageMonitorOpMode extends LinearOpMode {
    VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        waitForStart();

        while(!isStopRequested()) {
            telemetry.addData("12v monitor", voltageSensor.getVoltage()); //Battery voltage
            telemetry.update();
            sleep(500);
        }
    }
}
