package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.math.BigDecimal;
import java.math.MathContext;


public class ShooterController {

    public double veloTolerance = 0, shotInterval = 0, maxVelo = 0, pow = 0;
    ShooterInitializer sc;
    long lastPressed = 0, lastFlick = 0;

    public ShooterController(Class<? extends ShooterInitializer> sc, HardwareMap hwm, double veloTolerance, double shotInterval, double maxVelo) {
        this.veloTolerance = veloTolerance;
        this.shotInterval = shotInterval;
        this.maxVelo = maxVelo;
        try {
            this.sc = sc.newInstance();
            this.sc.init(hwm);
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    enum Step {
        FORWARD,
        BACKWARD
    }

    public void setPower(double power) {
        sc.setMotorEnabled();
        sc.spinUp(power);
        pow = power;
    }

    public void stopMotor() {
        sc.spinUp(0);
        sc.setMotorDisabled();
        pow = 0;
    }


    public void update(boolean shoot) {
        actPow = Math.abs(sc.getVelocity() / maxVelo);
        switch (step) {
            case FORWARD:
                if (System.currentTimeMillis() - lastFlick > shotInterval * .5) step = Step.BACKWARD;
                break;
            case BACKWARD:
                if(shoot && actPow > Math.abs(pow) - veloTolerance &&
                        actPow < Math.abs(pow) + veloTolerance &&
                        System.currentTimeMillis() - lastFlick > shotInterval &&
                        Math.abs(sc.getVelocity()) > 0) {
                    lastFlick = System.currentTimeMillis();
                    step = Step.FORWARD;
                    shots += 1;

                }
                break;
        }

        sc.setServo(step == Step.FORWARD ? ShooterInitializer.position.FORWARD : ShooterInitializer.position.BACKWARD);

    }

    public double getVelocity() {
        return sc.getVelocity();
    }

    public void setServo(ShooterInitializer.position position) {
        sc.setServo(position);
    }


}
