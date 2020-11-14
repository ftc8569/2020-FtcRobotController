package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Dev: WobbleGrabberTest", group = "Development")
@Disabled
public class WobbleGrabberTest extends OpMode {

    public Servo grabberServo;
    public DcMotorEx flipperMotor;
    long lastPressed = 0;

    public static double    grabberOpenPos   =  0,
                            grabberClosedPos =  0.33,
                            armStartPos = 0,
                            armUpPos = -183,
                            armForwardPos = -368;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, .5, 10);
    public static double pCoefficient = 10;

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
        flipperMotor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
        flipperMotor.setPositionPIDFCoefficients(pCoefficient);
        grabberServo = this.hardwareMap.servo.get("grabberServo");
        this.grabberServo.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() {
        if(gamepad1.a && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            grabberServo.setPosition(open ? grabberClosedPos : grabberOpenPos);
            open = !open;
        }

//        if(gamepad1.dpad_up && System.currentTimeMillis() - lastPressed > 500) {
//            lastPressed = System.currentTimeMillis();
//            currentPos += increment;
//            flipperMotor.setTargetPosition(currentPos);
//        } else if(gamepad1.dpad_down && System.currentTimeMillis() - lastPressed > 500) {
//            lastPressed = System.currentTimeMillis();
//            currentPos -= increment;
//            flipperMotor.setTargetPosition(currentPos);
//        }

        if         (gamepad1.dpad_up) currentPos = (int) armUpPos;
        else if  (gamepad1.dpad_left) currentPos = (int) armForwardPos;
        else if (gamepad1.dpad_right) currentPos = (int) armStartPos;
        flipperMotor.setTargetPosition(currentPos);

        telemetry.addData(
                "currentPos", currentPos
        );
        telemetry.addData("ServoPos", grabberServo.getPosition());
        telemetry.addData("Position PIDF", flipperMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));


    }
}
