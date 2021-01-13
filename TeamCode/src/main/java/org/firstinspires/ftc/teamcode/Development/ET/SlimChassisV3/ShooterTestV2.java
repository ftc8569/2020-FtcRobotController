package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;

@TeleOp
public class ShooterTestV2 extends OpMode {

    ShooterController sc;
    public static double maxVelocity = 2400,
            veloToleranceTower = .007,
            veloTolerancePowerShots = .004,
            shotInterval = 500;

    public static double pow = 0;

    long lastPressed = 0;

    public final int POWER_ADJUSTMENT_DELAY = 250; //the amount of time it has to wait before adjusting power of the shooter motor again again.


    @Override
    public void init() {
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, veloToleranceTower, shotInterval, maxVelocity);
    }

    @Override
    public void loop() {
        if((this.gamepad1.dpad_up || this.gamepad1.dpad_down) && System.currentTimeMillis() - lastPressed > POWER_ADJUSTMENT_DELAY) {
            pow += this.gamepad1.dpad_up ? .0125 : -.0125;
            lastPressed = System.currentTimeMillis();
        }

        sc.setPower(pow);

        sc.update(gamepad1.right_trigger > .05);

        if(gamepad1.a) {
            pow = -1;
        } else if (gamepad1.x) pow = 0;

        telemetry.addData("setPow:", pow);
        telemetry.addData("actualPower:", sc.getVelocity() / 2400.0);
        telemetry.addData("Shoot:", gamepad1.right_trigger > 0.5);

    }
}
