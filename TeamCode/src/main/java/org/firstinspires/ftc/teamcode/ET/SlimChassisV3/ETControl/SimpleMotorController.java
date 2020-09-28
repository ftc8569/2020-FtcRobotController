package org.firstinspires.ftc.teamcode.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Objects;

public class SimpleMotorController implements MotorController {
    protected ArrayList<String> motorNames = new ArrayList<>();
    protected ArrayList<DcMotorEx> motors = new ArrayList<>();
    HardwareMap hw;

    public SimpleMotorController(String[] devicenames, HardwareMap hwm) {
        hw = hwm;
        Collections.addAll(motorNames, devicenames);
        motorNames.forEach(mot -> motors.add(hw.get(DcMotorEx.class, mot)));
    }

    public SimpleMotorController(Class<? extends Enum<?>> devicenames, HardwareMap hwm) {
        hw = hwm;
        Collections.addAll(motorNames, getNames(devicenames));
        motorNames.forEach(mot -> motors.add(hw.get(DcMotorEx.class, mot)));
    }

    public static String[] getNames(Class<? extends Enum<?>> e) {
        return Arrays.stream(Objects.requireNonNull(e.getEnumConstants())).map(Enum::name).toArray(String[]::new); // stolen from stackoverflow
    }

    @Override
    public int findMotor(String motorname) throws IllegalArgumentException {
        if(motorNames.indexOf(motorname) != -1) return motorNames.indexOf(motorname);
        else throw new IllegalArgumentException("Could not find specified motor with name: " + motorname);
    }

    @Override
    public int findMotor(Enum motorname) throws IllegalArgumentException{
        if(motorNames.indexOf(motorname.name()) != -1) return motorNames.indexOf(motorname.name());
        else throw new IllegalArgumentException("Could not find specified motor with name: " + motorname);
    }

    @Override
    public void setMotor(String motorname, double pow) {
        if(findMotor(motorname) != -1) motors.get(findMotor(motorname)).setPower(pow);
    }

    @Override
    public void setMotor(int mIndex, double pow) {
        if(mIndex > -1 && mIndex < motors.size()) {
            motors.get(mIndex).setPower(pow);
        }
    }

    @Override
    public void setMotor(String[] motornames, double pow) {
        for (String motor : motornames) {

                motors.get(findMotor(motor)).setPower(pow);

        }
    }

    @Override
    public void setMotor(Enum motorname, double pow) {
        motors.get(findMotor(motorname)).setPower(pow);
    }


    @Override
    public void zeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        motors.forEach(motor -> motor.setZeroPowerBehavior(zpb));
    }

    @Override
    public void zeroPowerBehavior(String motorname, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(findMotor(motorname)).setZeroPowerBehavior(zpb);
    }

    @Override
    public void zeroPowerBehavior(Enum motorname, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(findMotor(motorname)).setZeroPowerBehavior(zpb);
    }

    @Override
    public void zeroPowerBehavior(int motorindex, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(motorindex).setZeroPowerBehavior(zpb);
    }

    @Override
    public void zeroPowerBehavior(String[] motornames, DcMotor.ZeroPowerBehavior zpb) {
        for (String motor : motornames) {

                motors.get(findMotor(motor)).setZeroPowerBehavior(zpb);

        }
    }


    @Override
    public void setDirection(DcMotorSimple.Direction dir) {
        motors.forEach(motor -> motor.setDirection(dir));
    }

    @Override
    public void setDirection(String motorname, DcMotorSimple.Direction dir) {
        motors.get(findMotor(motorname)).setDirection(dir);
    }

    @Override
    public void setDirection(Enum motorname, DcMotorSimple.Direction dir) {
        motors.get(findMotor(motorname)).setDirection(dir);
    }

    @Override
    public void setDirection(int motorindex, DcMotorSimple.Direction dir) {
        if(motorindex > -1 && motorindex < motors.size()) motors.get(motorindex).setDirection(dir);
        else throw new IndexOutOfBoundsException("Specified motor index " + motorindex + " does not exist.");
    }

    @Override
    public void setDirection(String[] motornames, DcMotorSimple.Direction dir) {
        for (String motor : motornames) {

                motors.get(findMotor(motor)).setDirection(dir);

        }
    }


    @Override
    public void reverse() {
        motors.forEach(motor -> {
            DcMotorSimple.Direction dir = motor.getDirection();
            if(dir == DcMotorSimple.Direction.FORWARD) motor.setDirection(DcMotorSimple.Direction.REVERSE);
            else if(dir == DcMotorSimple.Direction.REVERSE) motor.setDirection(DcMotorSimple.Direction.FORWARD);
        });
    }

    @Override
    public void reverse(String motorname) {

            DcMotorSimple.Direction dir = motors.get(findMotor(motorname)).getDirection();
            if(dir == DcMotorSimple.Direction.FORWARD) motors.get(findMotor(motorname)).setDirection(DcMotorSimple.Direction.REVERSE);
            else if(dir == DcMotorSimple.Direction.REVERSE) motors.get(findMotor(motorname)).setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void reverse(Enum motorname) {

            DcMotorSimple.Direction dir = motors.get(findMotor(motorname)).getDirection();
            if(dir == DcMotorSimple.Direction.FORWARD) motors.get(findMotor(motorname)).setDirection(DcMotorSimple.Direction.REVERSE);
            else if(dir == DcMotorSimple.Direction.REVERSE) motors.get(findMotor(motorname)).setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void reverse(int motorindex) {
        if(motorindex > -1 && motorindex < motors.size()) {
            DcMotorSimple.Direction dir = motors.get(motorindex).getDirection();
            if(dir == DcMotorSimple.Direction.FORWARD) motors.get(motorindex).setDirection(DcMotorSimple.Direction.REVERSE);
            else if(dir == DcMotorSimple.Direction.REVERSE) motors.get(motorindex).setDirection(DcMotorSimple.Direction.FORWARD);
        } else throw new IndexOutOfBoundsException("Specified motor index " + motorindex + " does not exist.");
    }

    @Override
    public void reverse(String[] motornames) {
        for (String motor : motornames) {
            if(findMotor(motor) != -1) {
                DcMotorSimple.Direction dir = motors.get(findMotor(motor)).getDirection();
                if(dir == DcMotorSimple.Direction.FORWARD) motors.get(findMotor(motor)).setDirection(DcMotorSimple.Direction.REVERSE);
                else if(dir == DcMotorSimple.Direction.REVERSE) motors.get(findMotor(motor)).setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

    }

}
