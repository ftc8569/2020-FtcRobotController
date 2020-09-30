package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.SimpleMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Objects;

public class FTCLibMotorController implements MotorController {
    protected ArrayList<String> motorNames = new ArrayList<>();
    public ArrayList<SimpleMotorEx> motors = new ArrayList<>();
    protected ArrayList<Double> motorCPRs = new ArrayList<>();
    protected HardwareMap hw;


    public FTCLibMotorController(String[] devicenames, HardwareMap hwm, double cpr) {
        hw = hwm;
        Collections.addAll(motorNames, devicenames);
        motorNames.forEach(mot -> {
            motors.add(new SimpleMotorEx(mot, hwm, cpr));
            motorCPRs.add(cpr);
        });
    }

    public FTCLibMotorController(Class<? extends Enum<?>> devicenames, HardwareMap hwm, double cpr) {
        hw = hwm;
        Collections.addAll(motorNames, getNames(devicenames));
        motorNames.forEach(mot -> {
            motors.add(new SimpleMotorEx(mot, hwm, cpr));
            motorCPRs.add(cpr);
        });
    }

    public FTCLibMotorController(String[] devicenames, HardwareMap hwm, double[] MotorCPRs) {
        hw = hwm;
        if(Objects.requireNonNull(devicenames).length == Objects.requireNonNull(MotorCPRs).length) {
            for(int i = 0; i < devicenames.length; i++) {
                motorNames.add(devicenames[i]);
                motorCPRs.add(MotorCPRs[i]);
                        motors.add(new SimpleMotorEx(devicenames[i], hwm, MotorCPRs[i]));
                }
            }
         else throw new IllegalArgumentException("Arrays are not the same length");
    }

    public FTCLibMotorController(Class<? extends Enum<?>> devicenames, HardwareMap hwm, double[] MotorCPRs) {
        hw = hwm;
        if(Objects.requireNonNull(devicenames.getEnumConstants()).length == Objects.requireNonNull(MotorCPRs).length) {
            for(int i = 0; i < getNames(devicenames).length; i++) {
                motorNames.add(getNames(devicenames)[i]);
                motorCPRs.add(MotorCPRs[i]);
                        motors.add(new SimpleMotorEx(getNames(devicenames)[i], hwm, MotorCPRs[i]));
            }
        } else throw new IllegalArgumentException("Enum " + devicenames.getName() + " and motor type array are not the same length");
    }

    public static String[] getNames(Class<? extends Enum<?>> e) {
        return Arrays.stream(Objects.requireNonNull(e.getEnumConstants())).map(Enum::name).toArray(String[]::new); // stolen from stackoverflow
    }

    @Override
    public int findMotor(String motorname) throws IllegalArgumentException {
        if(motorNames.indexOf(motorname) != -1) return motorNames.indexOf(motorname);
        else throw new IllegalArgumentException("Could not find specified motor.");
    }

    @Override
    public int findMotor(Enum motorname) throws IllegalArgumentException {
        if(motorNames.indexOf(motorname.name()) != -1) return motorNames.indexOf(motorname.name());
        else throw new IllegalArgumentException("Could not find specified motor.");
    }

    @Override
    public void setMotor(String motorname, double pow) {
        motors.get(findMotor(motorname)).set(pow);
    }

    @Override
    public void setMotor(int mIndex, double pow) {
            motors.get(mIndex).set(pow);
    }

    @Override
    public void setMotor(String[] motornames, double pow) {
        for (String motor : motornames) {

            motors.get(findMotor(motor)).set(pow);

        }
    }

    @Override
    public void setMotor(Enum motorname, double pow) {
        motors.get(findMotor(motorname)).set(pow);
    }

    @Override
    public void zeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        motors.forEach(motor -> motor.setZeroPowerBehavior(zpb == DcMotor.ZeroPowerBehavior.BRAKE ? MotorEx.ZeroPowerBehavior.BREAK : MotorEx.ZeroPowerBehavior.valueOf(zpb.name())));
    }

    @Override
    public void zeroPowerBehavior(String motorname, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(findMotor(motorname)).setZeroPowerBehavior(zpb == DcMotor.ZeroPowerBehavior.BRAKE ? MotorEx.ZeroPowerBehavior.BREAK : MotorEx.ZeroPowerBehavior.valueOf(zpb.name()));
    }

    @Override
    public void zeroPowerBehavior(Enum motorname, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(findMotor(motorname)).setZeroPowerBehavior(zpb == DcMotor.ZeroPowerBehavior.BRAKE ? MotorEx.ZeroPowerBehavior.BREAK : MotorEx.ZeroPowerBehavior.valueOf(zpb.name()));
    }

    @Override
    public void zeroPowerBehavior(int motorindex, DcMotor.ZeroPowerBehavior zpb) {
        motors.get(motorindex).setZeroPowerBehavior(zpb == DcMotor.ZeroPowerBehavior.BRAKE ? MotorEx.ZeroPowerBehavior.BREAK : MotorEx.ZeroPowerBehavior.valueOf(zpb.name()));
    }

    @Override
    public void zeroPowerBehavior(String[] motornames, DcMotor.ZeroPowerBehavior zpb) {
        for (String motor : motornames) {
                motors.get(findMotor(motor)).setZeroPowerBehavior(zpb == DcMotor.ZeroPowerBehavior.BRAKE ? MotorEx.ZeroPowerBehavior.BREAK : MotorEx.ZeroPowerBehavior.valueOf(zpb.name()));
        }
    }

    @Override
    public void setDirection(DcMotorSimple.Direction dir) {
        motors.forEach(motor -> motor.setInverted(dir != DcMotorSimple.Direction.FORWARD));
    }

    @Override
    public void setDirection(String motorname, DcMotorSimple.Direction dir) {
        motors.get(findMotor(motorname)).setInverted(dir != DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void setDirection(Enum motorname, DcMotorSimple.Direction dir) {
        motors.get(findMotor(motorname)).setInverted(dir != DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void setDirection(int motorindex, DcMotorSimple.Direction dir) {
        motors.get(motorindex).setInverted(dir != DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void setDirection(String[] motornames, DcMotorSimple.Direction dir) {
        for (String motor : motornames) {

            motors.get(findMotor(motor)).setInverted(dir != DcMotorSimple.Direction.FORWARD);
        }
    }

    @Override
    public void reverse() {
        motors.forEach(motor -> {
            motor.setInverted(!motor.getInverted());
        });
    }

    @Override
    public void reverse(String motorname) {
            motors.get(findMotor(motorname)).setInverted(!motors.get(findMotor(motorname)).getInverted());
    }

    @Override
    public void reverse(Enum motorname) {
            motors.get(findMotor(motorname)).setInverted(!motors.get(findMotor(motorname)).getInverted());
    }

    @Override
    public void reverse(int motorindex) {
            motors.get(motorindex).setInverted(!motors.get(motorindex).getInverted());
    }

    @Override
    public void reverse(String[] motornames) {
        for (String motor : motornames) {
                motors.get(findMotor(motor)).setInverted(!motors.get(findMotor(motor)).getInverted());
        }
    }

    public double getCPR(String motorName) {
        return motors.get(findMotor(motorName)).COUNTS_PER_REV;
    }

    public double getCPR(Enum motorName) {
        return motors.get(findMotor(motorName)).COUNTS_PER_REV;
    }
}
