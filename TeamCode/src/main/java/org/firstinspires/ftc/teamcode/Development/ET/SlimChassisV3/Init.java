package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Objects;


public class Init extends OpMode { // should be obsolete now that SimpleMotorController exists.
   public ArrayList<String> motorNames = new ArrayList<>();
   public ArrayList<DcMotorEx> motors = new ArrayList<>();

    public void stopAllMotors() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public ArrayList<DcMotorEx> addMotors(Class<?> devicenames, ArrayList<String> deviceNames) {
        ArrayList<DcMotorEx> output = new ArrayList<>();

        for (motornames motor : Objects.requireNonNull(motornames.class.getEnumConstants())) {
            output.add(this.hardwareMap.get(DcMotorEx.class, motor.name()));
        }
        return output;
    }

    public int findMotor(motornames motorName) {
        boolean nameFound = false;
        int motorIndex = -1;
        for(int i = 0; i < Objects.requireNonNull(motornames.class.getEnumConstants()).length; i++) { //TODO: make it so that if it finds two motors of the same name it yells
            if(motornames.class.getEnumConstants()[i] == motorName) {
                nameFound = true;
                motorIndex = i;
            }
        }
        if(nameFound) return motorIndex;
        else return -1;

    }

    public void motorPower(motornames motorName, double power) {
       if(findMotor(motorName) >= 0) {motors.get(findMotor(motorName)).setPower(power);}
       else { telemetry.addData("That's not good", "It looks like our search for your motor was unsuccessful. This means that the name was wrong or your motorname list is wrong. Please fix it"); }
    }

    @Override
    public void init() {
        motors = addMotors(motornames.class, motorNames);
        telemetry.addData("Motornames", motorNames);
        telemetry.addData("motors", motors);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors.get(findMotor(motornames.FrontLeftMotor)).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(findMotor(motornames.BackLeftMotor)).setDirection(DcMotorSimple.Direction.REVERSE);
    }


enum motornames  {FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor}

    @Override
    public void loop() {

    }

}
