package org.firstinspires.ftc.teamcode.PreProduction.Depreciated;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShotPowers;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.SlimChassisV3Controller;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;

/**
 * older version of scrimmageteleop with no automation
 */
@Disabled
@Config
@TeleOp(name = "Pre: ScrimmageTeleOp1", group = "Pre-Production")
public class ScrimmageTeleOp extends OpMode {
    FieldOrientedDrive fod;
    ShooterController sc;
    DoubleSupplier djx = () -> gamepad1.left_stick_x * movementMultiplier, djy = () -> -gamepad1.left_stick_y * movementMultiplier, dr = () -> Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2) * movementMultiplier, heading;
    RevIMU imu;

    public Servo grabberServo;
    public DcMotorEx flipperMotor;

    DcMotorEx topMotor, bottomMotor;

    long lastPressed = 0, lastPressed2 = 0, lastPressed3 = 0, lastPressed4 = 0, shooterChanged = 0;

    public static double armOffset = 0,
            headingOffset = 0;

    public static double    grabberOpenPos   =  0,
                            grabberClosedPos =  0.33,
                            armStartPos = ScrimmageAutoV2.armStartPos + 360,
                            armUpPos = ScrimmageAutoV2.armUpPos + 360,
                            armForwardPos = ScrimmageAutoV2.armForwardPos + 360,
                            shooterDefaultPower = -.710,
                            currentPower = 0,
                            movementMultiplier = 1;

    public static ShotPowers pows = new ShotPowers(-.71, -.7025, -.71); // powers for straight on shooting.
    //                                                                      Angle shooting needs higher powers
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, 1, 20);
    public static double pCoefficient = 15;

    public boolean  open = true,
                    spun = false,
                    changedManual = false;

    int currentPos = 0;

    int rings = 0, oldRings = 0;

    ArrayList<Double> ringDists = new ArrayList<>();


    intakeDirections intakeDirection = intakeDirections.Stop, gamepadDirection = intakeDirections.Stop;

    double maxVelo = 0;

    double armPos = 0;

    double offset = 0;

    DistanceSensor dist;

    public void init() {
        Collections.addAll(ringDists, 3.5, 2.6, 1.5, 1.05);
        imu = new RevIMU(hardwareMap);
        imu.init();
        heading = () -> imu.getHeading() + offset;

        fod = new FieldOrientedDrive(SlimChassisV3Controller.class, hardwareMap, heading, djx, djy, dr);
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .005, 1000, 2400);
        MotorBulkRead.MotorBulkMode(LynxModule.BulkCachingMode.MANUAL, hardwareMap);

        flipperMotor = this.hardwareMap.get(DcMotorEx.class, "flipperMotor");
//        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        flipperMotor.setTargetPositionTolerance(20);
//        flipperMotor.setTargetPosition(currentPos);
//        flipperMotor.setPower(.125);
        flipperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flipperMotor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
//        flipperMotor.setPositionPIDFCoefficients(pCoefficient);

        grabberServo = this.hardwareMap.servo.get("grabberServo");
        this.grabberServo.setDirection(Servo.Direction.REVERSE);

        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dist = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
    }

    public void loop() {
        if(!fod.isEnabled()) fod.enable();
        MotorBulkRead.clearCache();
        fod.control();

        sc.update(gamepad1.right_trigger >= .25);

        if(gamepad2.left_bumper && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            grabberServo.setPosition(open ? grabberClosedPos : grabberOpenPos);
            open = !open;
        }

        if(rings != oldRings) {
            shooterChanged = System.currentTimeMillis();
            oldRings = rings;
        }

        if(gamepad2.x && System.currentTimeMillis() - lastPressed2 > 1000 && !gamepad1.start) {
            spun = !spun;
            lastPressed2 = System.currentTimeMillis();
//            if(spun) changedManual = true;
        }
//        else if(!changedManual && System.currentTimeMillis() - shooterChanged > 500 && rings == 3) {
//                spun = true;
//                shooterChanged = System.currentTimeMillis();
//        }
        else if(rings == 0 && System.currentTimeMillis() - shooterChanged > 100) {
            spun = false;
            shooterChanged = System.currentTimeMillis();
            changedManual = false;
        }

        double x = dist.getDistance(DistanceUnit.INCH);
        double answer = ringDists.get(0);
        double current = Double.MAX_VALUE;
        if(x < 100) {
            for (Double value : ringDists) {
                if (Math.abs(value - x) < current) {
                    answer = value;
                    current = Math.abs(value - x);
                }
            }
        } else answer = -1;

        rings = ringDists.indexOf(answer);

//        rings = 3 - ((sc.getShots() % 3));

//        rings = 3;

        if(spun) {
            movementMultiplier = .5;
            switch(rings) {
                case 3:
                    currentPower = pows.pow1;
                    sc.setPower(currentPower);
                    break;

                case 2:
                default:
                    currentPower = pows.pow2;
                    sc.setPower(currentPower);
                    break;

                case 1:
                    currentPower = pows.pow3;
                    sc.setPower(currentPower);
                    break;
            }
        } else {
            currentPower = 0;
            sc.setPower(currentPower);
            movementMultiplier = 1;
        }

        if(Math.abs(gamepad2.right_stick_y) > .05) flipperMotor.setPower(gamepad2.right_stick_y * .25);
        else flipperMotor.setPower(0);
//        if         (gamepad2.dpad_up) currentPos = (int) armUpPos;
//        else if  (gamepad2.dpad_left) currentPos = (int) armForwardPos;
//        else if (gamepad2.dpad_right) currentPos = (int) armStartPos;
//        if(!flipperMotor.isBusy()) {
//            flipperMotor.setPower(0);
//        } else {
//            flipperMotor.setPower(.125);
//        }
//        flipperMotor.setTargetPosition(currentPos);

        if(gamepad2.y && !gamepad1.start) intakeDirection = intakeDirections.In;
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

        if(gamepad1.y && System.currentTimeMillis() - lastPressed3 > 10000) imu.reset();


        if(Math.abs(sc.getVelocity()) > maxVelo) maxVelo = Math.abs(sc.getVelocity());

//        telemetry.addData("currentPos", currentPos);
//        telemetry.addData("ServoPos", grabberServo.getPosition());
//        telemetry.addData("Position PIDF", flipperMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
//        telemetry.addData("Heading", imu.getHeading());
//        telemetry.addData("Right Trigger", gamepad1.right_trigger);
//        telemetry.addData("MotorPower", topMotor.getPower() + ", " + bottomMotor.getPower());
//        telemetry.addData("CurrentVelo", sc.getVelocity());
//        telemetry.addData("MaxVelo", maxVelo);
//        telemetry.addData("armPos", flipperMotor.getCurrentPosition());
        telemetry.addData("Distance", dist.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Current Power", currentPower);
        telemetry.addData("Rings", (rings));
    }

    public enum intakeDirections {
        In,
        Stop,
        Out
    }
}
