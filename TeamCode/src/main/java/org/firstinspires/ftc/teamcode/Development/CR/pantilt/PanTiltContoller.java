package org.firstinspires.ftc.teamcode.Development.CR.pantilt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PanTiltContoller {
    public static double PANSTRAIGHTFORWARD = 0;
    public static double TILTSTRAIGHTFORWARD = 0;
    public static double SERVORANGEDEGREES = 270;
    private Servo panServo, tiltServo;

    public  PanTiltContoller(HardwareMap hardwareMap) {
        panServo = hardwareMap.get(Servo.class, "panservo");
        tiltServo = hardwareMap.get(Servo.class, "tiltservo");
    }
    public void Initialize() {
        setPanPosition(PANSTRAIGHTFORWARD);
        setTiltPosition(TILTSTRAIGHTFORWARD);
    }
    public void setPosition(double panAngle, double tiltAngle) {
        setTiltPosition(tiltAngle);
        setPanPosition(panAngle);
    }
    // +180 (up) to -180 (down) subject to servo limits
    public void setTiltPosition(double tiltAngle) {
        tiltServo.setPosition( getServoPositionFromAngle(tiltAngle));
    }
    // +180 (right) to -180 (left) subject to servo limits
    public void setPanPosition(double panAngle) {
        panServo.setPosition( getServoPositionFromAngle(panAngle));
    }
    public double getCurrentPanAngle() {
        return (panServo.getPosition() - 0.5) * SERVORANGEDEGREES;
    }
    public double getCurrentTiltAngle() {
        return (tiltServo.getPosition() - 0.5) * SERVORANGEDEGREES;
    }
    public static double getMinimumAngle() {
        return  -(SERVORANGEDEGREES/2);
    }
    public static double getMaximumAngle() {
        return  (SERVORANGEDEGREES/2);
    }
    private static double getServoPositionFromAngle(double angle) {
        double clippedAngle = Math.min(Math.max(angle, getMinimumAngle()), getMaximumAngle());
        return  0.5 + clippedAngle/SERVORANGEDEGREES; // range [0,1]
    }

}
