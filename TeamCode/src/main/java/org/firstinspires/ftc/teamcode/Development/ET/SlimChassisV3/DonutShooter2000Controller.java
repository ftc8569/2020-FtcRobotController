package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;

public class DonutShooter2000Controller extends ShooterInitializer {
    public DcMotorEx flywheel;
    Servo flicker;

    public final double
            SHOOTERSERVOBACK = .75,
            SHOOTERSERVOFORWARD = 0;


    @Override
    public void init(HardwareMap hw) {
        flywheel = hw.get(DcMotorEx.class, "ShooterMotorFront");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(10,3,0,0); //defaults

        flicker = hw.servo.get("ShooterServo");
//        flicker.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void spinUp(double power) {
        flywheel.setPower(power);
    }

    @Override
    public void setServo(position position) {
        if(position == ShooterInitializer.position.FORWARD) {
            flicker.setPosition(SHOOTERSERVOFORWARD);
        } else flicker.setPosition(SHOOTERSERVOBACK);
    }

    @Override
    public void setServo(double position) {
        flicker.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return flywheel.getVelocity();
    }


}
