/*package org.firstinspires.ftc.teamcode.Development.CR.python.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="PythonTestOpMode", group = "CR-Python")
public class PythonTestOpMode extends LinearOpMode {
    public PythonTestOpMode() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if(!Python.isStarted())
            Python.start(new AndroidPlatform(hardwareMap.appContext));

        Python pyInstance = Python.getInstance();

        PyObject module = pyInstance.getModule("testmodule2");

        String received = module.callAttr("test", "").toString();

        telemetry.addData("received", received);
        telemetry.update();

        waitForStart();

    }
}*/
