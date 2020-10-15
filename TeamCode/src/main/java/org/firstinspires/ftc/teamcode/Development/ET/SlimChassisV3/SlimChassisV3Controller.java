package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.DriveTrainController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.SimpleMotorController;

public class SlimChassisV3Controller extends DriveTrainController {
    SimpleMotorController smc;
    @Override
    public void initialize(HardwareMap hw) {
    smc = new SimpleMotorController(motornames, hw);
    }

    @Override
    public void setPowers(double[] pows) {
        for(int i = 0; i < motornames.length; i++) {
            smc.setMotor(i, pows[i]);
        }
    }

    @Override
    public void setPowers(double fl, double fr, double bl, double br) {
        double[] pows = {fl, fr, bl, br};
        setPowers(pows);
    }

    protected String[] motornames = {
            "FrontLeft",
            "FrontRight",
            "BackLeft",
            "BackRight"
    };
}
