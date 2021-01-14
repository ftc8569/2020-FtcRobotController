package org.firstinspires.ftc.teamcode.PreProduction;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Development.CR.vision.PipelineStages;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingDetectionEnum;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShotPowers;
import org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.Waypoints.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.DoubleSupplier;


//@Disabled
@Config
@TeleOp(name = "Pre: ScrimmageTeleOp", group = "Pre-Production")
public class ScrimmageTeleOp extends OpMode {
    FieldOrientedDrive fod;
    ShooterController sc;
    SampleMecanumDrive drive;
    DoubleSupplier djx = () -> gamepad1.left_stick_x * movementMultiplier, djy = () -> -gamepad1.left_stick_y * movementMultiplier, dr = () -> Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2) * movementMultiplier, heading;
//    RevIMU imu;

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
                            powerShotPower = -.715,
                            currentPower = 0,
                            movementMultiplier = 1,
                            maxVelocity = 2400,
                            veloToleranceTower = .007,
                            veloTolerancePowerShots = .004,
                            shotInterval = 500;

    public static Pose2d    powerShot1 = new Pose2d(36.5, -42, Math.toRadians(0)),
                            powerShot2 = new Pose2d(36.5, -39, Math.toRadians(0)),
                            powerShot3 = new Pose2d(36.5, -36, Math.toRadians(0));

    public static ShotPowers pows = new ShotPowers(-.71, -.7025, -.71); // powers for straight on shooting.
    //                                                                      Angle shooting needs higher powers
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5, 3, 0, 20);
    public static double pCoefficient = 15;

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(30, 4.5, 0, 0);

    public boolean  open = false,
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

    Pose2d poseEstimate;

    int lastShots = 0;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    enum pathPieces {
        DRIVE,
        DRIVING,
        SHOOT
    }

    pathPieces pathMode = pathPieces.DRIVE;
    int repeats = 0;

    long lastShot = 0;

    enum positions {
        POWERSHOT1,
        POWERSHOT2,
        POWERSHOT3,
        STOP
    }

    positions designatedPos = positions.POWERSHOT1;


    RevBlinkinLedDriver led;

    long LEDChanged = 0;

    public static RevBlinkinLedDriver.BlinkinPattern blink = RevBlinkinLedDriver.BlinkinPattern.GREEN;

//    FtcDashboard dashboard;

    public static volatile PipelineStages PIPELINESTAGE = PipelineStages.OUTPUTWITHBOUNDINGRECT;

    OpenCvCamera webcam;

    RingFinderPipeline pipeline;




    public void init() {
        telemetry.addData(">", "Initialization started");
        Collections.addAll(ringDists, 3.5, 2.6, 1.5, 1.05);

        webcam:
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            pipeline = new RingFinderPipeline(webcam, telemetry);
            pipeline.pipelineStageToDisplay = PIPELINESTAGE;
            webcam.setPipeline(pipeline);

            FtcDashboard.getInstance().startCameraStream(webcam, 10);

            webcam.openCameraDeviceAsync(() -> webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
        }

        DriveConstants.maxVel /= 2;
        DriveConstants.maxAccel /= 2;
        DriveConstants.maxAngleVel /= 2;
        DriveConstants.maxAngleAccel /= 2;

//        dashboard = FtcDashboard.getInstance();

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive.FOLLOWER_HEADING_TOLERANCE = Math.toRadians(.125);
        SampleMecanumDrive.FOLLOWER_POSITION_TOLERANCE = .06125;
        SampleMecanumDrive.FOLLOWER_TIMEOUT = 2;

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        LEDChanged = System.currentTimeMillis();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);


        heading = () -> Math.toDegrees(drive.getPoseEstimate().getHeading()) + offset;

        fod = new FieldOrientedDrive(heading, djx, djy, dr);
        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, veloToleranceTower, shotInterval, maxVelocity);
        sc.setPIDF(shooterCoeffs);
        MotorBulkRead.MotorBulkMode(LynxModule.BulkCachingMode.MANUAL, hardwareMap);

        flipperMotor = this.hardwareMap.get(DcMotorEx.class, "flipperMotor");
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperMotor.setTargetPositionTolerance(20);
        flipperMotor.setTargetPosition(currentPos);
        flipperMotor.setPower(.175);
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipperMotor.setMotorDisable();
        flipperMotor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
        flipperMotor.setPositionPIDFCoefficients(pCoefficient);

        grabberServo = this.hardwareMap.servo.get("grabberServo");
        this.grabberServo.setDirection(Servo.Direction.REVERSE);

        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        dist = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");

        while(System.currentTimeMillis() - LEDChanged < 1000);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        telemetry.addData(">", "Initialization completed");

    }

    public void loop() {
//        if(!fod.isEnabled()) fod.enable();
        MotorBulkRead.clearCache();
//        fod.control();
        drive.update();

        //just here for the sake of easy updates with dashboard
//        sc.setPIDF(shooterCoeffs);


        Pose2d poseEstimate = drive.getPoseEstimate();

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

        switch (currentMode) {
            case DRIVER_CONTROL:
                double[] wheelPows = fod.calculate();

                drive.setMotorPowers(wheelPows[0], wheelPows[2], wheelPows[3], wheelPows[1]);

                if(rings == 0 && System.currentTimeMillis() - shooterChanged > 100) {
                    spun = false;
                    shooterChanged = System.currentTimeMillis();
                    changedManual = false;
                }

                if(spun) {
                    movementMultiplier = .5;
                    switch(rings) {
                        case 3:
                            currentPower = pows.getPow1();
                            sc.setPower(currentPower);
                            break;

                        case 2:
                        default:
                            currentPower = pows.getPow2();
                            sc.setPower(currentPower);
                            break;

                        case 1:
                            currentPower = pows.getPow3();
                            sc.setPower(currentPower);
                            break;
                    }
                } else {
                    currentPower = 0;
                    sc.setPower(currentPower);
                    sc.stopMotor();
                    movementMultiplier = 1;
                }

                if (gamepad1.right_bumper && gamepad1.left_bumper) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL
                    pathMode = pathPieces.DRIVE;
                    designatedPos = positions.POWERSHOT1;

                    drive.setPoseEstimate(new Pose2d());

                    currentMode = Mode.AUTOMATIC_CONTROL;
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                    sc.setVeloTolerance(veloTolerancePowerShots);

                }
                break;
            case AUTOMATIC_CONTROL:

                // If x is pressed, we break out of the automatic following
                if (gamepad1.x) {
                    currentMode = Mode.DRIVER_CONTROL;
                    spun = false;
                    sc.stopMotor();
                    sc.resetShots();
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                    sc.setVeloTolerance(veloToleranceTower);

                }

                switch (pathMode) {
                    case SHOOT:
                        if (sc.getShots() <= 0) {
                            sc.update(true);
                            lastShot = System.currentTimeMillis();
                        } else {
//                            spun = false;
                            if(designatedPos == positions.STOP) {
                                if(System.currentTimeMillis() - lastShot > 100) {
                                    currentMode = Mode.DRIVER_CONTROL;
                                    spun = false;
                                    sc.stopMotor();
                                    sc.resetShots();
                                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                                    sc.setVeloTolerance(veloToleranceTower);
                                }
                            } else {
                                sc.resetShots();
                                pathMode = pathPieces.DRIVE;
                            }

                        }
                        break;

                    case DRIVE:
                        poseEstimate = drive.getPoseEstimate();

                        switch (designatedPos) {
                            case POWERSHOT1:
                                Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                        .splineToLinearHeading(powerShot1, 0)
                                        .build();

                                drive.followTrajectoryAsync(traj1);
                                pathMode = pathPieces.DRIVING;
                                designatedPos = positions.POWERSHOT2;
                                break;
                            case POWERSHOT2:
                                Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                                        .splineToLinearHeading(powerShot2, 0)
                                        .build();

                                drive.followTrajectoryAsync(traj2);
                                pathMode = pathPieces.DRIVING;
                                designatedPos = positions.POWERSHOT3;
                                break;
                            case POWERSHOT3:
                                Trajectory traj3 = drive.trajectoryBuilder(poseEstimate)
                                        .splineToLinearHeading(powerShot3, 0)
                                        .build();

                                drive.followTrajectoryAsync(traj3);
                                pathMode = pathPieces.DRIVING;
                                designatedPos = positions.STOP;
                                break;

                            case STOP:
                                currentMode = Mode.DRIVER_CONTROL;
                                pathMode = pathPieces.DRIVE;
                                designatedPos = positions.POWERSHOT1;
                        }
                        break;

                    case DRIVING:
                        if(!drive.isBusy()) {
                            pathMode = pathPieces.SHOOT;
                            spun = true;
                            sc.setPower(-.71);
                        }
                }
        }

        sc.update(gamepad1.right_trigger >= .25);
        if(sc.getShots() != lastShots) {
            pipeline.CaptureImage();
            lastShots = sc.getShots();
        }

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





        if(Math.abs(gamepad2.right_stick_y) > .05 || Math.abs(gamepad2.right_stick_x) > .05) {
            double desAng = Math.atan2(-gamepad2.right_stick_y, -gamepad2.right_stick_x);
            double desDegrees = Math.toDegrees(desAng); //for my own comfort
            final double CPR = 753.2;
            double TPD = CPR / 360;
            double desTicks = TPD * desDegrees;
//                    >= 0 ? desDegrees : desDegrees + 180;

            flipperMotor.setMotorEnable();
            flipperMotor.setTargetPosition((int) desTicks);
            telemetry.addData("DesiredDegrees", desDegrees);
        } else {
            flipperMotor.setMotorDisable();
        }


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

        if(gamepad1.y && System.currentTimeMillis() - lastPressed3 > 10000) drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));


        if(Math.abs(sc.getVelocity()) > maxVelo) maxVelo = Math.abs(sc.getVelocity());

        if(currentMode == Mode.AUTOMATIC_CONTROL) {
            telemetry.addData("AUTOMATIC CONTROL INITIATED", "!!!");
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velocity", sc.getVelocity());

//        dashboard.sendTelemetryPacket(packet);

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
        telemetry.addData("Set Power", Math.abs(currentPower));
        telemetry.addData("CurrentPower", Math.abs(sc.getVelocity() / maxVelocity));
        telemetry.addData("Rings", (rings));
        telemetry.addData("pose", poseEstimate);
        telemetry.addData("mode", currentMode);
        telemetry.addData("ArmPosition", flipperMotor.getCurrentPosition());
//        telemetry.addData("PDIF", sc.getPIDF());
    }

    public enum intakeDirections {
        In,
        Stop,
        Out
    }
}
