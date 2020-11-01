package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.DriveTrainController;

import java.util.function.DoubleSupplier;

/**
 * A easy to use, importable field oriented drive function with the ability to either calculate
 * motor powers or control motors for you. Also supports double suppliers because they are just
 * better.
 * All headings are in degrees counter-clockwise of the zero reference (I think.)
 * All powers are given in the order FrontLeft, FrontRight, BackLeft, BackRight.
 * You need to create your own DriveTrainController in order to use the control features.
 *
 * @see FieldOrientedDrive
 * @see DriveTrainController
 */

//TODO: Find out whether we are getting heading clockwise or counter.

public class FieldOrientedDrive {
    protected boolean ds, hw, enabled = false; //whether or not you are using a double supplier
    //and whether or not you give the method the motors so that it can directly control them.

    DoubleSupplier theta, x, y, r;
    HardwareMap hwm;
    double cx, cy, cr, ct;
    DriveTrainController obj;

    /**
     * @param dtc A user-defined controller with the functionality to initialize and apply powers to
     *            whatever drivetrain you have.
     * @param hwm The hardware Map of the motors you are controlling.
     * @param theta A double supplier that gives rotation in degrees counter-clockwise from the 0
     *              reference.
     * @param x A double supplier that gives joystick movement in the x (horizontal) axis on the
     *          controller.
     * @param y A double supplier that gives joystick movement in the Y (vertical) axis on the
     *          controller. Remember that the Y axis is inverted on the logitech F310 (?) Gamepad,
     *          so you will need to flip that.
     * @param r A double supplier that gives rotational speed clockwise. Depending on your scheme
     *          this may be the right stick or the triggers or twist on the stick but I don't think
     *          any FTC controllers support that.
     * @throws InstantiationException Stuff to be able to pass the controller class.
     * @throws IllegalAccessException Stuff to be able to pass the controller class.
     */

    public FieldOrientedDrive(Class<? extends DriveTrainController> dtc, HardwareMap hwm, DoubleSupplier theta, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) throws InstantiationException, IllegalAccessException {
        ds = true;
        hw = true;
        this.hwm = hwm;
        this.x = x;
        this.y = y;
        this.r = r;
        this.theta = theta;
        DriveTrainController obj = dtc.newInstance();
        obj.initialize(hwm);
    }

    /**
     * @param dtc A user-defined controller with the functionality to initialize and apply powers to
     *            whatever drivetrain you have.
     * @param hwm The hardware Map of the motors you are controlling.
     * @throws InstantiationException Stuff to be able to pass the controller class.
     * @throws IllegalAccessException Stuff to be able to pass the controller class.
     */
    public FieldOrientedDrive(Class<? extends DriveTrainController> dtc, HardwareMap hwm) throws InstantiationException, IllegalAccessException {
        ds = false;
        hw = true;
        this.hwm = hwm;
        obj = dtc.newInstance();
        obj.initialize(hwm);
    }

    /**
     * @param theta A double supplier that gives rotation in degrees counter-clockwise from the 0
     *              reference.
     * @param x A double supplier that gives joystick movement in the x (horizontal) axis on the
     *          controller.
     * @param y A double supplier that gives joystick movement in the Y (vertical) axis on the
     *          controller. Remember that the Y axis is inverted on the logitech F310 (?) Gamepad,
     *          so you will need to flip that.
     * @param r A double supplier that gives rotational speed clockwise. Depending on your scheme
     *          this may be the right stick or the triggers or twist on the stick but I don't think
     *          any FTC controllers support that.
     */
    public FieldOrientedDrive(DoubleSupplier theta, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        ds = true;
        hw = false;
        this.x = x;
        this.y = y;
        this.r = r;
        this.theta = theta;
    }

    public FieldOrientedDrive() {
        ds = false;
        hw = false;
    }

    public void control() {
        if(!ds) {
            throw new UnsupportedOperationException("You must use either double suppliers or give arguments");
        } else if(!hw) {
            throw new UnsupportedOperationException("You must either supply a drivetrain controller or power the motors on your own");
        } else if(!enabled) {
            throw new UnsupportedOperationException("Motor control not enabled. Please call the enable() method.");
        } else obj.setPowers(calculate(x.getAsDouble(), r.getAsDouble(), y.getAsDouble(), theta.getAsDouble()));
    }

    public void control(double x, double y, double r, double theta) {
        if(hw) {
            if(enabled) {
                obj.setPowers(calculate(x, r, y, theta));
            } else throw new UnsupportedOperationException("Motor control not enabled. Please call the enable() method.");
        } else throw new UnsupportedOperationException("You must either supply a drivetrain controller or power the motors on your own");
    }

    public double[] calculate() {
        if(!ds) {
            throw new UnsupportedOperationException("You must use either double suppliers or give arguments");
        } else {
            return calculate(x.getAsDouble(), y.getAsDouble(), r.getAsDouble(), theta.getAsDouble());
        }
    }

    public double[] calculate(double x, double y, double r, double theta) {
        double  temp   = x * Math.cos(Math.toRadians(theta)) - y * Math.sin(Math.toRadians(theta));
        y  = x * Math.sin(Math.toRadians(theta)) + y * Math.cos(Math.toRadians(theta));
        x = temp;

        double frontLeft  = x + y + r;
        double frontRight = x - y - r;
        double backLeft   = x - y + r;
        double backRight  = x + y - r;

        double max = Math.abs(frontLeft);
        if(Math.abs(frontRight) > max) max = Math.abs(frontRight);
        if(Math.abs(backLeft) > max) max = Math.abs(backLeft);
        if(Math.abs(backRight) > max) max = Math.abs(backRight);

        if(max > 1) {
            frontRight /= max;
            frontLeft  /= max;
            backRight  /= max;
            backLeft   /= max;
        }
        return new double[]{frontLeft, frontRight, backLeft, backRight};
    }


    public void enable() {enabled = true;}
    public void disable() {enabled = false;}
    public boolean isEnabled() {return enabled;}

}
