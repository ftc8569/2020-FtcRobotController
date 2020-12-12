package org.firstinspires.ftc.teamcode.Development.CR.drive;

import org.firstinspires.ftc.teamcode.Development.ET.util.Encoder;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.net.PortUnreachableException;

public class OdometryPod {
    private static final double WHEEL_RADIUS = 1.89; // in
    private static final double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private RevThroughBoreEncoder encoder;
    private Pose2d pose;

    public OdometryPod(HardwareMap hardwareMap, String motorName, Pose2d pose) {
        this(new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, motorName)), pose);
    }
    public OdometryPod(RevThroughBoreEncoder encoder, Pose2d pose) {
        this.encoder = encoder;
        this.pose = pose;
    }
    public Encoder getEncoder() {
        return this.encoder;
    }
    public Pose2d getPose() {
        return  this.pose;
    }
    public double getPosition() {
        return encoderTicksToInches(this.encoder.getCurrentPosition());
    }
    public double getVelocity() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return encoderTicksToInches(this.encoder.getCurrentVelocity());
    }

    private static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / RevThroughBoreEncoder.TICKS_PER_REV;
    }

}
