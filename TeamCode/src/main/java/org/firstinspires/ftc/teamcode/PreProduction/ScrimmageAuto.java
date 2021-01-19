package org.firstinspires.ftc.teamcode.PreProduction;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Development.CR.vision.PipelineStages;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingDetectionEnum;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.AutoPaths;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShotPowers;
import org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageTeleOp;
import org.firstinspires.ftc.teamcode.PreProduction.Waypoints.FourRingWaypoints;
import org.firstinspires.ftc.teamcode.PreProduction.Waypoints.OneRingWaypoints;
import org.firstinspires.ftc.teamcode.PreProduction.Waypoints.ZeroRingWaypoints;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAngleAccel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAngleVel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxVel;

//@Config
//@Disabled
@Autonomous(name = "Pre: ScrimmageAuto", group = "Pre-Production", preselectTeleOp = "Pre: ScrimmageTeleOp")
public class ScrimmageAuto extends LinearOpMode {

    public static volatile int HMIN = 60, SMIN = 100, VMIN = 50;
    public static volatile int HMAX = 120, SMAX = 250, VMAX = 250;
    public static volatile int NORINGUPPER = 200, ONERINGUPPER = 2000;
    public static volatile PipelineStages PIPELINESTAGE = PipelineStages.OUTPUTWITHBOUNDINGRECT;

    public static double shotInterval = 325;

    RingDetectionEnum rings = RingDetectionEnum.UNKNOWN;

    OpenCvCamera webcam;
    /* Above is all ring finder stuff*/


    public Servo grabberServo;
    public DcMotorEx flipperMotor;
    long lastPressed = 0;

    public static ShooterController sc;
    public static double    grabberOpenPos   =  ScrimmageTeleOp.grabberOpenPos,
            grabberClosedPos =  ScrimmageTeleOp.grabberClosedPos,
            armStartPos = 0,
            armUpPos = -193,
            armForwardPos = -314;
    public static PIDFCoefficients pidfCoefficients = ScrimmageTeleOp.pidfCoefficients;
    public static double pCoefficient = ScrimmageTeleOp.pCoefficient;

    public static ShotPowers powOffset = new ShotPowers(.00, -.00, -.0075);

    public static ShotPowers shooterPower = new ShotPowers(ScrimmageTeleOp.pows.getPow1() - powOffset.getPow1(), ScrimmageTeleOp.pows.getPow2() - powOffset.getPow2(), ScrimmageTeleOp.pows.getPow3() - powOffset.getPow3());

    public static PIDFCoefficients pidf = new PIDFCoefficients(100, 4, 0.025, 0);



    public boolean open = true;

    int currentPos = 0;

    RevBlinkinLedDriver led;

    long LEDChanged = 0;

    public static RevBlinkinLedDriver.BlinkinPattern blink = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    public static DcMotorEx topMotor, bottomMotor;


    public static Pose2d startPose;

    public AutoPaths paths;

    public final int targetPositionTolerance = 20;

    public static double slowFactor = 50;

    public ZeroRingWaypoints zero;

    public OneRingWaypoints one;

    public FourRingWaypoints four;

    public RingFinderPipeline pipeline;




    @SuppressWarnings({"StatementWithEmptyBody", "UnusedLabel"})
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Starting initialization, please wait...");
        telemetry.update();

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        LEDChanged = System.currentTimeMillis();

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

        org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageTeleOp.armOffset = 0;
        org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageTeleOp.headingOffset = 0;



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        startPose = new Pose2d(9, 32, Math.toRadians(-0)); //1ft, 7in, -69
        drive.setPoseEstimate(ScrimmageAuto.startPose);

        wobbleGrabber:
        {
            flipperMotor = this.hardwareMap.get(DcMotorEx.class, "flipperMotor");
            flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flipperMotor.setTargetPositionTolerance(targetPositionTolerance);
            flipperMotor.setTargetPosition(currentPos);
            flipperMotor.setPower(.125);
            flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flipperMotor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
            flipperMotor.setPositionPIDFCoefficients(pCoefficient);
            flipperMotor.setMotorDisable();

            grabberServo = this.hardwareMap.servo.get("grabberServo");
            this.grabberServo.setDirection(Servo.Direction.REVERSE);
            grabberServo.setPosition(grabberClosedPos);
        }

        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .007, shotInterval, 2400);
        sc.setPIDF(pidf);
        sc.setServo(ShooterInitializer.position.BACKWARD);

        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zero = new ZeroRingWaypoints();
        zero.init(drive);

        one = new OneRingWaypoints();
        one.init(drive);

        four = new FourRingWaypoints();
        four.init(drive);



        while(System.currentTimeMillis() - LEDChanged < 1000);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);




//        constantly takes readings until started
        while (!isStarted() && !isStopRequested()) {
            rings = pipeline.RingsDetected;
            telemetry.addData(">", "Initialization completed.");
            telemetry.addData("Rings", rings);
            telemetry.addData("Calibration Status:", drive.imu.getCalibrationStatus());
            telemetry.update();
        }


        waitForStart();

        if(isStopRequested()) return;

        //start with one wobble preloaded
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);


        switch (rings) {
            case ZERO: {
                paths = zero;
                drive.followTrajectory(paths.toShoot);

                //Label so that you can collapse it on the left to make readability easier.
                // Consider it a poor man's method and an even poorer man's class
                shooting:
                {
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow1());

//                314

                    //sleep to give the shooter time to spin up
                    sleep(1250);
                    while (!sc.canShoot()) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow2());
                    sleep(350);
                    while (!sc.canShoot()) ;


                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());
                    sleep(350);
                    while (!sc.canShoot()) ;

                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                }

                drive.followTrajectory(paths.toDrop);

                dropping:
                {
                    flipperMotor.setMotorEnable();
                    flipperMotor.setTargetPosition((int) armForwardPos);
                    currentPos = (int) armForwardPos;

                    while (!withinTolerance(flipperMotor.getCurrentPosition(), targetPositionTolerance, currentPos))
                        ;
                    sleep(250);

                    flipperMotor.setMotorDisable();

                    //set grabber to open position
                    grabberServo.setPosition(grabberOpenPos);
                    sleep(300);
                }

                drive.followTrajectory(paths.toPick);

                grabbing:
                {
                    //set grabber to Closed position
                    grabberServo.setPosition(grabberClosedPos);
                    sleep(500);
                }

                drive.followTrajectory(paths.toDrop2);

                dropping2:
                {
                    //set grabber to open position
                    grabberServo.setPosition(grabberOpenPos);
                    sleep(500);
                }

                drive.followTrajectory(paths.toLine);

                grabberServo.setPosition(grabberClosedPos);

                sleep(500);


                break;
            }

            case ONE: {
                paths = one;
                ifIAddALabelWillYouStopYellingAtMeIntelliJ: {
                drive.followTrajectory(paths.toShoot);
                }

                shooting:
                {
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow1());

//                314

                    //sleep to give the shooter time to spin up
                    sleep(1250);
                    while (!sc.canShoot()) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow2());
                    sleep(350);
                    while (!sc.canShoot()) ;


                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());
                    sleep(350);
                    while (!sc.canShoot()) ;

                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                }

                drive.followTrajectory(paths.toDrop);

                dropping:
                {
                    flipperMotor.setMotorEnable();
                    flipperMotor.setTargetPosition((int) armForwardPos);
                    currentPos = (int) armForwardPos;

                    while (!withinTolerance(flipperMotor.getCurrentPosition(), targetPositionTolerance, currentPos)) ;
                    sleep(250);

                    flipperMotor.setMotorDisable();

                    //set grabber to open position
                    grabberServo.setPosition(grabberOpenPos);
                    sleep(300);
                }

                topMotor.setPower(1);
                bottomMotor.setPower(1);

                drive.followTrajectory(paths.toRings);

                drive.leftRear.setPower(0);
                drive.leftFront.setPower(0);
                drive.rightRear.setPower(0);
                drive.rightFront.setPower(0);

                sleep(350);

                drive.followTrajectory(paths.toRingsSlow);

                drive.followTrajectory(paths.toPick);

                grabbing:
                {
                    //set grabber to Closed position
                    grabberServo.setPosition(grabberClosedPos);
                    sleep(500);
                }

                topMotor.setPower(0);
                bottomMotor.setPower(0);

                drive.followTrajectory(paths.toDrop2);

                dropping2:
                {
                    //set grabber to open position
                    grabberServo.setPosition(grabberOpenPos);
                    sleep(500);
                }

                flipperMotor.setMotorEnable();
                flipperMotor.setTargetPosition((int) armStartPos);

                grabberServo.setPosition(grabberClosedPos);

                drive.followTrajectory(paths.toShoot2);
                flipperMotor.setMotorDisable();

                shooting2:
                {
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());

                    //sleep to give the shooter time to spin up
                    sleep(1250);
                    while (!sc.canShoot()) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);


                }

                flipperMotor.setMotorEnable();
                flipperMotor.setTargetPosition((int) armForwardPos);

                drive.followTrajectory(paths.toLine);

                flipperMotor.setMotorDisable();



                break;

            }

            case FOUR: {
                paths = four;
                ifIAddALabelWillYouStopYellingAtMeIntelliJPleaseThankYou: {
                    drive.followTrajectory(paths.toShoot);
                }

                shooting:
                {
                    //spins up the shooter so that it is prepared to shoot
//                    sc.setPower(shooterPower.getPow1());

//                314

                    //sleep to give the shooter time to spin up
//                    sleep(1250);
                    while (!sc.canShoot()) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow2());
                    sleep(350);
                    while (!sc.canShoot()) ;


                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());
                    sleep(350);
                    while (!sc.canShoot()) ;

                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                }

                drive.followTrajectory(paths.toDrop);

                dropping:
                {
                    flipperMotor.setMotorEnable();
                    flipperMotor.setTargetPosition((int) armForwardPos);
                    currentPos = (int) armForwardPos;

                    while (!withinTolerance(flipperMotor.getCurrentPosition(), targetPositionTolerance, currentPos));
//                    sleep(250);

                    flipperMotor.setMotorDisable();

                    //set grabber to open position
                    grabberServo.setPosition(grabberOpenPos);
                    sleep(250);
                }

//                flipperMotor.setMotorEnable();
//                flipperMotor.setTargetPosition((int) armStartPos);

                topMotor.setPower(1);
                bottomMotor.setPower(1);

                drive.followTrajectory(paths.toRings);

                grabberServo.setPosition(grabberClosedPos);


                drive.rightFront.setPower(0);
                drive.rightRear.setPower(0);
                drive.leftFront.setPower(0);
                drive.leftRear.setPower(0);

                sleep(350);

//                flipperMotor.setMotorDisable();

//                sleep(100);
                drive.followTrajectory(paths.toRingsSlow);
//                sleep(100);
                drive.followTrajectory(paths.toRingsSlow2);
//                sleep(100);
                drive.followTrajectory(paths.toRingsSlow3);

                sc.setPower(shooterPower.getPow1());
                drive.followTrajectory(paths.toShoot2);

                topMotor.setPower(0);
                bottomMotor.setPower(0);

                shooting2:
                {
                    //spins up the shooter so that it is prepared to shoot
//                    sc.setPower(shooterPower.getPow1());

//                314

                    //sleep to give the shooter time to spin up
//                    sleep(1250);
                    long time = System.currentTimeMillis();
                    while (!sc.canShoot() || System.currentTimeMillis() - time < 400) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow2());
                    sleep(350);
                    while (!sc.canShoot()) ;


                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());
                    sleep(350);
                    while (!sc.canShoot()) ;

                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                }

                topMotor.setPower(1);
                bottomMotor.setPower(1);

                drive.followTrajectory(paths.toLastRing);

                drive.followTrajectory(paths.toShoot3);

                shooting3:
                {
                    //spins up the shooter so that it is prepared to shoot
//                    sc.setPower(shooterPower.getPow1());

//                314

                    //sleep to give the shooter time to spin up
//                    sleep(1250);
                    long time = System.currentTimeMillis();
                    while (!sc.canShoot() || System.currentTimeMillis() - time < 400) ;

                    //shooting code. Basically setting servo forwards and back repeatedly
                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow2());
                    sleep(350);
                    while (!sc.canShoot()) ;


                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shooterPower.getPow3());
                    sleep(350);
                    while (!sc.canShoot()) ;

                    sc.setServo(ShooterInitializer.position.FORWARD);
                    sleep(350);
                    sc.setServo(ShooterInitializer.position.BACKWARD);
                    sleep(100);

                    //stops shooter motor and intake if it is moving
                    sc.stopMotor();
                    topMotor.setPower(0);
                    bottomMotor.setPower(0);
                }

                drive.followTrajectory(paths.toLine);




                break;
            }
        }

        //only in here temporarily to make sure the program doesn't exit too early.
//        while (!isStopRequested()) idle();

    }


    public boolean withinTolerance(double value, double tolerance, double setPoint) {
        return value >= setPoint - tolerance && value < setPoint + tolerance;
    }

}



