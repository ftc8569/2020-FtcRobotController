package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Dev: TelemetryPacketTest", group = "Development")
public class TelemetryPacketTest extends OpMode {

    FtcDashboard dashboard;
    @Override
    public void init() {
        FtcDashboard.start();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);


    }

    @Override
    public void loop() {

    }
}
