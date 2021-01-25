package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;

// First Shot: -26.9
@Config
@TeleOp
public class ShooterTestV2 extends OpMode {

    ShooterController sc;
    public static double maxVelocity = 2400,
            veloToleranceTower = .007,
            veloTolerancePowerShots = .006,
            shotInterval = 525;

    public static double pow = 0;

    long lastPressed = 0, lastPressed2 = 0, lastPressed3 = 0;

    boolean enabled = false;

    public final int POWER_ADJUSTMENT_DELAY = 250; //the amount of time it has to wait before adjusting power of the shooter motor again again.

    enum mode {
        TOWER, POWERSHOTS
    }

    mode Mode = mode.TOWER;

    FtcDashboard dashboard;

    public static PIDFCoefficients pidf = new PIDFCoefficients(100, 4, 0.025, 0);
    //HEavy flywheel, 100, 4, .025, 0
    //light slywheel, 35, 4, 2, 0



    @Override
    public void init() {
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, veloToleranceTower, shotInterval, maxVelocity);
        sc.setPIDF(pidf);
        FtcDashboard.start();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        sc.setShotInterval(shotInterval);
        if((this.gamepad1.dpad_up || this.gamepad1.dpad_down) && System.currentTimeMillis() - lastPressed > POWER_ADJUSTMENT_DELAY) {
            pow += this.gamepad1.dpad_up ? .0125 : -.0125;
            lastPressed = System.currentTimeMillis();
        }
        if(sc.isMotorEnabled()) {
            sc.setPower(pow);
        }

        sc.update(gamepad1.right_trigger > .25);

        if(gamepad1.a) {
            pow = -1;
        } else if (gamepad1.x && System.currentTimeMillis() - lastPressed2 > 500) {
            if(sc.isMotorEnabled()) sc.setMotorDisabled(); else sc.setMotorEnabled();
            lastPressed2 = System.currentTimeMillis();
        }

        if(gamepad1.y && System.currentTimeMillis() - lastPressed3 > 500) {
            switch (Mode) {

                case TOWER:
                    Mode = mode.POWERSHOTS;
                    sc.setVeloTolerance(veloTolerancePowerShots);
                    break;
                case POWERSHOTS:
                    Mode = mode.TOWER;
                    sc.setVeloTolerance(veloToleranceTower);
                    break;
            }
            lastPressed3 = System.currentTimeMillis();
        }

        sc.setPIDF(pidf);

        telemetry.addData("setPow:", -pow);
        telemetry.addData("actualPower:", sc.getActPow());
        telemetry.addData("Shoot:", gamepad1.right_trigger > 0.5);
        telemetry.addData("Mode:", Mode);
        telemetry.addData("MotorEnabled:", sc.isMotorEnabled());

//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("setPow:", pow * -1);
//        packet.put("actualPower:", sc.getActPow());
//        dashboard.sendTelemetryPacket(packet);

    }
}
