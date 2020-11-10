package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Imagine this like DriveTrainController but for a shooter. It is assumed the shooter is flywheel
 * based and that it uses a servo to push the rings into the flywheel.
 * @see DriveTrainController
 */
public abstract class ShooterInitializer {

    /**
     * Just needs to initialize the motor and servo you are using however you need them (runmode,
     * direction, etc).
     * @param hw the HardwareMap the motors and servos are a part of.
     */
    public abstract void init(HardwareMap hw);

    /**
     * Sets the flywheel to the designated power
     * @param power the power to be set to, between -1 and 1, as powers do
     */
    public abstract void spinUp(double power);

    /**
     * should make the servo actuate to the set position. Forward should be pushing the ring into
     * the flywheel and back should be away from the flywheel
     */
    public abstract void setServo(position position);

    public abstract double getVelocity();

    public enum position {
        FORWARD,
        BACKWARD
    }

}
