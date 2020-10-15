package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A simple abstract class to be extended by one specific to your robot. It requires you to define
 * three classes. One that initializes all of your motors in whatever way you like, and one
 * (maybe one with two parameter types) that sets all of your motors to powers given. For a four
 * wheeled drivetrain, powers will always be given in the order:
 * FrontLeft, FrontRight, BackLeft, BackRight
 *
 * @see FieldOrientedDrive
 */
public abstract class DriveTrainController {


    /**
     * A user defined method that initializes their motors correctly so that they can be used by
     * another class, such as {@link FieldOrientedDrive}.
     * There is not anything special here, just init them how you normally would for your other
     * programs.
     *
     * @param hw the hardware map of the motors you are initializing. Everything else should be hard
     *           coded, but I don't think there is a nice way to get this. So it will be necessary
     *           for you to pass it in your OpMode.
     * @see FieldOrientedDrive
     */

    // need to define a method that initializes your motors
    public abstract void initialize(HardwareMap hw);

    //need to define a method that powers your motors based on an input power.
    public abstract void setPowers(double[] pows);
    public abstract void setPowers(double fl, double fr, double bl, double br);

}
