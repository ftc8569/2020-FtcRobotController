package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.Importable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorInitializer;

import java.util.function.DoubleSupplier;

public class FieldOrientedDrive {
    protected boolean ds, hw; //whether or not you are using a double supplier
    //and whether or not you give the method the motors so that it can directly control them.

    DoubleSupplier theta, x, y, r;
    HardwareMap hwm;

    public FieldOrientedDrive(Class<? extends MotorInitializer> mi, DoubleSupplier theta, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
    ds = true;
    hw = true;


    }

    public FieldOrientedDrive(HardwareMap hwm, String[] motorNames) {
    ds = false;
    hw = true;

    }

    public FieldOrientedDrive(DoubleSupplier theta, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
    ds = true;
    hw = false;

    }

    public FieldOrientedDrive() {
    ds = false;
    hw = false;

    }

    public void update() {
        if(!ds) {
            throw new UnsupportedOperationException("You must use either double suppliers or give arguments");
        } else;
    }

}
