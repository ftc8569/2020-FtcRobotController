package org.firstinspires.ftc.teamcode.Development.CR.drive;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Development.CR.util.Encoder;

public class RevThroughBoreEncoder extends Encoder {
    public static final double TICKS_PER_REV = 8192;

    public RevThroughBoreEncoder(DcMotorEx motor, NanoClock clock) {
        super(motor, clock);
    }

    public RevThroughBoreEncoder(DcMotorEx motor) {
        super(motor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentVelocity() {
        //  Because the Rev Through Bore encoder velocity can exceed 32767 counts / second
        //  change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return getCorrectedVelocity();
    }
}
