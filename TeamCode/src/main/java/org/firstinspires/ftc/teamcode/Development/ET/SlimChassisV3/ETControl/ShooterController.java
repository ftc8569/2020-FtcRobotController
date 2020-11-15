package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class ShooterController {

    double veloTolerance, shotInterval, maxVelo, pow = 0;
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

    public void setPower(double power) {
        sc.spinUp(power);
        pow = power;
    }

    public void stopMotor() {
        sc.spinUp(0);
        pow = 0;
    }

    public void update(boolean shoot) {
        if (System.currentTimeMillis() - lastFlick < shotInterval * .6)
            sc.setServo(ShooterInitializer.position.FORWARD);
        else if (shoot && Math.abs(sc.getVelocity() / maxVelo) > Math.abs(pow) - veloTolerance && Math.abs(sc.getVelocity() / maxVelo) < Math.abs(pow) + veloTolerance && System.currentTimeMillis() - lastFlick > shotInterval) {
            lastFlick = System.currentTimeMillis();
            sc.setServo(ShooterInitializer.position.FORWARD);
        } else sc.setServo(ShooterInitializer.position.BACKWARD);
    }

    public double getVelocity() {
        return sc.getVelocity();
    }

    public void setServo(ShooterInitializer.position position) {
        sc.setServo(position);
    }


}
