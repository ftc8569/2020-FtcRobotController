package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.DriveTrainController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.SimpleMotorController;

public class SlimChassisV3Controller extends DriveTrainController {
    public SimpleMotorController smc;
    @Override
    public void initialize(HardwareMap hw) {
        smc = new SimpleMotorController(motornames, hw);
        smc.motors.forEach(mot -> mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        smc.motors.forEach(mot -> mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        smc.reverse(new String[]{"FrontLeftMotor", "BackLeftMotor"});
        smc.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            "FrontLeftMotor",
            "FrontRightMotor",
            "BackLeftMotor",
            "BackRightMotor"
    };
}
