package org.firstinspires.ftc.teamcode.PreProduction;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.SlimChassisV3Controller;

import java.util.function.DoubleSupplier;

//@Disabled
@TeleOp(name = "Pre: ScrimmageTeleOp", group = "Pre-`Production")
public class ScrimmageTeleOp extends OpMode {
    FieldOrientedDrive fod;
    ShooterController sc;
    DoubleSupplier djx = () -> gamepad1.left_stick_x, djy = () -> -gamepad1.left_stick_y, dr = () -> gamepad1.right_stick_x, heading;
    RevIMU imu;

    public Servo grabberServo;
    public DcMotorEx flipperMotor;

    DcMotorEx topMotor, bottomMotor;

    long lastPressed = 0, lastPressed2 = 0;

    public static double    grabberOpenPos   =  0,
                            grabberClosedPos =  0.33,
                            armStartPos = 0,
                            armUpPos = -183,
                            armForwardPos = -368,
                            shooterDefaultPower = -.710;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, .5, 10);
    public static double pCoefficient = 10;

    public boolean  open = true,
                    spun = false;

    int currentPos = 0;

    intakeDirections intakeDirection = intakeDirections.Stop, gamepadDirection = intakeDirections.Stop;

    double maxVelo = 0;

    public void init() {
        imu = new RevIMU(hardwareMap);
        imu.init();
        heading = () -> imu.getHeading();

        fod = new FieldOrientedDrive(SlimChassisV3Controller.class, hardwareMap, heading, djx, djy, dr);
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .025, 750, 2400);
        MotorBulkRead.MotorBulkMode(LynxModule.BulkCachingMode.MANUAL, hardwareMap);

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

        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        if(!fod.isEnabled()) fod.enable();
        MotorBulkRead.clearCache();
        fod.control();

        sc.update(gamepad1.right_trigger >= .25);

        if(gamepad2.left_trigger >= .5 && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            grabberServo.setPosition(open ? grabberClosedPos : grabberOpenPos);
            open = !open;
        }

        if(gamepad1.a && System.currentTimeMillis() - lastPressed2 > 500 && !gamepad1.start) {
            sc.setPower(spun ? 0 : shooterDefaultPower);
            spun = !spun;
            lastPressed2 = System.currentTimeMillis();
        }

        if         (gamepad2.dpad_up) currentPos = (int) armUpPos;
        else if  (gamepad2.dpad_left) currentPos = (int) armForwardPos;
        else if (gamepad2.dpad_right) currentPos = (int) armStartPos;
        flipperMotor.setTargetPosition(currentPos);

        if(gamepad2.y) intakeDirection = intakeDirections.In;
        if(gamepad2.b && !gamepad1.start) intakeDirection = intakeDirections.Stop;
        if(gamepad2.a && !gamepad1.start) intakeDirection = intakeDirections.Out;

        if(intakeDirection == intakeDirections.In) {
            topMotor.setPower(1);
            bottomMotor.setPower(1);
        } else if(intakeDirection == intakeDirections.Out) {
            topMotor.setPower(-1);
            bottomMotor.setPower(-1);
        } else {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }


        if(Math.abs(sc.getVelocity()) > maxVelo) maxVelo = Math.abs(sc.getVelocity());
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("ServoPos", grabberServo.getPosition());
        telemetry.addData("Position PIDF", flipperMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Heading", imu.getHeading());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("MotorPower", topMotor.getPower() + ", " + bottomMotor.getPower());
        telemetry.addData("CurrentVelo", sc.getVelocity());
        telemetry.addData("MaxVelo", maxVelo);
    }

    public enum intakeDirections {
        In,
        Stop,
        Out
    }
}
