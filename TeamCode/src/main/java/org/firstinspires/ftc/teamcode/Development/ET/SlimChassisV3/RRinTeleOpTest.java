package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShotPowers;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.Waypoints.PoseStorage;

import java.util.function.DoubleSupplier;

//@Disabled
@Config
@TeleOp(name = "Dev: RRinTeleOpTest", group = "Development")
public class RRinTeleOpTest extends OpMode {
    FieldOrientedDrive fod;
    SampleMecanumDrive drive;
    ShooterController sc;
    DoubleSupplier djx = () -> gamepad1.left_stick_x, djy = () -> -gamepad1.left_stick_y, dr = () -> Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2), heading;
    RevIMU imu;

    public Servo grabberServo;
    public DcMotorEx flipperMotor;

    DcMotorEx topMotor, bottomMotor;

    long lastPressed = 0, lastPressed2 = 0, lastPressed3 = 0, lastPressed4 = 0;

    public static double armOffset = 0,
            headingOffset = 0;

    public static double    grabberOpenPos   =  0,
                            grabberClosedPos =  0.33,
                            armStartPos = ScrimmageAutoV2.armStartPos + 360,
                            armUpPos = ScrimmageAutoV2.armUpPos + 360,
                            armForwardPos = ScrimmageAutoV2.armForwardPos + 360,
                            shooterDefaultPower = -.710,
                            currentPower = 0;

    public static ShotPowers pows = new ShotPowers(-.725, -.6975, -.68);
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, 1, 20);
    public static double pCoefficient = 15;

    public boolean  open = true,
                    spun = false;

    int currentPos = 0;

    public static Pose2d startingPose = new Pose2d();

    intakeDirections intakeDirection = intakeDirections.Stop, gamepadDirection = intakeDirections.Stop;

    double maxVelo = 0;

    double armPos = 0;

    DistanceSensor dist;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        imu = new RevIMU(hardwareMap);
        imu.init();
        heading = () -> imu.getHeading() - headingOffset;

        fod = new FieldOrientedDrive(heading, djx, djy, dr);
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .025, 750, 2400);
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
//        if(!fod.isEnabled()) fod.enable();
        MotorBulkRead.clearCache();
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();

        switch (currentMode) {
            case DRIVER_CONTROL:
                double[] wheelPows = fod.calculate();

                drive.setMotorPowers(wheelPows[0], wheelPows[2], wheelPows[3], wheelPows[1]);

                if (gamepad1.a) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                            .splineToLinearHeading(new Pose2d(-10, -44, Math.toRadians(-6)), 90)
                                    .build();

                    drive.followTrajectoryAsync(traj1);

                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (gamepad1.x) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }

        sc.update(gamepad1.right_trigger >= .25);

        if(gamepad2.left_bumper && System.currentTimeMillis() - lastPressed > 500) {
            lastPressed = System.currentTimeMillis();
            grabberServo.setPosition(open ? grabberClosedPos : grabberOpenPos);
            open = !open;
        }

        if(gamepad1.a && System.currentTimeMillis() - lastPressed2 > 500 && !gamepad1.start) {
            spun = !spun;
            lastPressed2 = System.currentTimeMillis();
        }

        if(spun) {
            switch((sc.getShots() % 3) + 1) {
                case 1:
                    currentPower = pows.pow1;
                    sc.setPower(currentPower);
                    break;

                case 2:
                    currentPower = pows.pow2;
                    sc.setPower(currentPower);
                    break;

                case 3:
                    currentPower = pows.pow3;
                    sc.setPower(currentPower);
                    break;
            }
        } else {
            currentPower = 0;
            sc.setPower(currentPower);
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

        if(gamepad1.y && System.currentTimeMillis() - lastPressed3 > 1000) imu.reset();


        if(Math.abs(sc.getVelocity()) > maxVelo) maxVelo = Math.abs(sc.getVelocity());

        telemetry.addData("currentPos", currentPos);
        telemetry.addData("ServoPos", grabberServo.getPosition());
        telemetry.addData("Position PIDF", flipperMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Heading", imu.getHeading());
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("MotorPower", topMotor.getPower() + ", " + bottomMotor.getPower());
        telemetry.addData("CurrentVelo", sc.getVelocity());
        telemetry.addData("MaxVelo", maxVelo);
        telemetry.addData("armPos", flipperMotor.getCurrentPosition());
        telemetry.addData("Distance", dist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Current Power", currentPower);
        telemetry.addData("Shot #", (sc.getShots() % 3) + 1);
    }

    public enum intakeDirections {
        In,
        Stop,
        Out
    }
}
