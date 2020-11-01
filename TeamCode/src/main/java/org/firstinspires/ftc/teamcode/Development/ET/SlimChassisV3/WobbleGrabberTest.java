package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dev: WobbleGrabberTest", group = "Development")
public class WobbleGrabberTest extends OpMode {

    public Servo grabberServo;
    public DcMotorEx flipperMotor;
    long lastPressed = 0;

    public static double    grabberOpenPos   =  0,
                            grabberClosedPos =  0.33;
    public boolean open = true;

    int currentPos = 0;
    double increment = (537.6 / 32);
    public void init() {
        flipperMotor = this.hardwareMap.get(DcMotorEx.class, "flipperMotor");
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperMotor.setTargetPosition(currentPos);
        flipperMotor.setPower(.125);
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberServo = this.hardwareMap.servo.get("grabberServo");
        this.grabberServo.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        if(gamepad1.a && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            grabberServo.setPosition(open ? grabberClosedPos : grabberOpenPos);
            open = !open;
        }

        if(gamepad1.dpad_up && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            currentPos += increment;
            flipperMotor.setTargetPosition(currentPos);
        } else if(gamepad1.dpad_down && System.currentTimeMillis() - lastPressed > 500) {
            currentPos -= increment;
            flipperMotor.setTargetPosition(currentPos);
        }

        telemetry.addData(
                "currentPos", currentPos
        );


    }
}
