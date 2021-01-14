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
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageTeleOp;
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

        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .01, 750, 2400);
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

                grabberServo.setPosition(grabberClosedPos);


                drive.followTrajectory(paths.toLine);

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

                flipperMotor.setMotorEnable();
                flipperMotor.setTargetPosition((int) armStartPos);

                grabberServo.setPosition(grabberClosedPos);

                topMotor.setPower(1);
                bottomMotor.setPower(1);

                drive.followTrajectory(paths.toRings);

                drive.rightFront.setPower(0);
                drive.rightRear.setPower(0);
                drive.leftFront.setPower(0);
                drive.leftRear.setPower(0);

                sleep(350);

                flipperMotor.setMotorDisable();

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
        while (!isStopRequested()) idle();

    }
//
//    public class RingFinderPipeline extends OpenCvPipeline
//    {
//        private FtcDashboard dashboard;
//        boolean viewportPaused;
//        // these are set for 320x240 pixels at approx 30 in from camera
//
//        public  RingFinderPipeline() {
//            super();
//            dashboard = FtcDashboard.getInstance();
//            dashboard.setTelemetryTransmissionInterval(25);
//        }
//
//
//        /*
//         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
//         * highly recommended to declare them here as instance variables and re-use them for
//         * each invocation of processFrame(), rather than declaring them as new local variables
//         * each time through processFrame(). This removes the danger of causing a memory leak
//         * by forgetting to call mat.release(), and it also reduces memory pressure by not
//         * constantly allocating and freeing large chunks of memory.
//         */
//        private Mat hsvImage = new Mat();
//        private Mat maskImage = new Mat();
//        private Mat blurImage = new Mat();
//        private Rect BoundingRectangle = null;
//        Scalar lowerHSVBound = new Scalar(HMIN, SMIN, VMIN);
//        Scalar upperHSVBound = new Scalar(HMAX, SMAX, VMAX);
//        Scalar measuredLowerHSVBound = lowerHSVBound.clone();
//        Scalar measuredUpperHSVBound = upperHSVBound.clone();
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile RingDetectionEnum RingsDetected = RingDetectionEnum.UNKNOWN;
//        public volatile PipelineStages PipelineStageToDisplay = PipelineStages.OUTPUTWITHBOUNDINGRECT;
//        public volatile String HSVSampleMetrics = "";
//        public volatile int measuredArea = 0;
///*
//        @Override
//        public void init(Mat firstFrame)
//        {
//            Imgproc.cvtColor(firstFrame, hsvImage, Imgproc.COLOR_BGR2HSV);
//            Scalar lower = new Scalar(6, 155, 70);
//            Scalar upper = new Scalar(67, 255, 255);
//            Core.inRange(hsvImage, lower, upper, maskImage);
//            Imgproc.medianBlur(maskImage, blurImage, 11);
//        }
//*/
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            try {
//                // Here we could possible crop the image to speed processing.  If we do, we need to do so in the init as well
//                //Rect cropRect = new Rect(83, 1017, 642, 237);
//                //Mat cropImage = input.submat(cropRect);
//                Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);
//
//                // refresh the lower/upper bounds so that this can work via FTC Dashboard
//                lowerHSVBound = new Scalar(HMIN, SMIN, VMIN);
//                upperHSVBound = new Scalar(HMAX, SMAX, VMAX);
//
//                Core.inRange(hsvImage, lowerHSVBound, upperHSVBound, maskImage);
//                Imgproc.medianBlur(maskImage, blurImage, 11);
//                ArrayList<MatOfPoint> contours = new ArrayList<>();
//                Mat hierarchy = new Mat();
//                Imgproc.findContours(blurImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//                if(contours.size() > 0) {
//                    // find the largest contour by area (this can't be assumed to be the first one in the list
//                    double maxContourArea = 0D;
//                    MatOfPoint maxContour = null;
//                    for(int i = 0; i< contours.size(); i++) {
//                        MatOfPoint contour = contours.get(i);
//                        double contourArea = Imgproc.contourArea(contour);
//                        if(contourArea > maxContourArea) {
//                            maxContourArea = contourArea;
//                            maxContour = contour;
//                        }
//                    }
//                    // maxContour should never be null...but somehow it was and was throwing an exception...
//                    if(maxContour != null) BoundingRectangle = Imgproc.boundingRect(maxContour);
//                }
//                else BoundingRectangle = null;
//
//                if(BoundingRectangle == null) {
//                    RingsDetected = RingDetectionEnum.ZERO;
//                    measuredArea = 0;
//                }
//                else {
//                    measuredArea = BoundingRectangle.width * BoundingRectangle.height;
//                    if(measuredArea <= NORINGUPPER ) RingsDetected = RingDetectionEnum.ZERO;
//                    else if (measuredArea <= ONERINGUPPER) RingsDetected = RingDetectionEnum.ONE;
//                    else RingsDetected = RingDetectionEnum.FOUR;
//
//                    // Draw a simple box around the rings
//                    Scalar rectColor = new Scalar(0, 255, 0);
//                    Imgproc.rectangle(input, BoundingRectangle, rectColor, 4);
//                    int baseline[]={0};
//                    Size textSize = Imgproc.getTextSize(RingsDetected.toString(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,2, baseline);
//                    int margin = 2;
//                    Point textOrigin = new Point(BoundingRectangle.x + BoundingRectangle.width/2 - textSize.width/2, BoundingRectangle.y - textSize.height - margin);
//                    Imgproc.putText(input, RingsDetected.toString(), textOrigin, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
//                }
//
//                switch (PipelineStageToDisplay) {
//                    case MASKIMAGE:
//                        return  maskImage;
//                    case BLURIMAGE:
//                        return  blurImage;
//                    case CONVERT2HSV:
//                        // when showing hsv, put a small box on the screen and
//                        // make a 100x100 square in the middle of the frame
//
//                        int width = 100;
//                        int height = 100;
//                        int x = (input.width() - width)/2;
//                        int y = (input.height() - height)/2;
//
//                        Rect sampleRect = new Rect(x,y,width,height);
//                        Imgproc.rectangle(hsvImage, sampleRect, new Scalar(0, 255, 0), 4);
//
//                        // now calculate the mean and std of the sample area
//                        Mat sampleImage = hsvImage.submat(sampleRect);
//                        MatOfDouble mean = new MatOfDouble();
//                        MatOfDouble std = new MatOfDouble();
//                        Core.meanStdDev(sampleImage, mean, std);
//                        double[] meanArray = mean.toArray();
//                        double[] stdArray = std.toArray();
//                        int[] hsvMean = new int[3];
//                        int[] hsvStd = new int[3];
//                        for(int i=0 ;i<3; i++){
//                            hsvMean[i] = (int)meanArray[i];
//                            hsvStd[i] = (int)stdArray[i];
//                        }
//
//                        HSVSampleMetrics = String.format("hsv mean (%d, %d,%d), std (%d, %d,%d)", hsvMean[0] , hsvMean[1], hsvMean[2], hsvStd[0], hsvStd[1], hsvStd[2]);
//                        return hsvImage;
//                    case OUTPUTWITHBOUNDINGRECT:
//                    case INPUT:
//                    default:
//                        return input;
//                }
//            }
//            catch (Exception ex){
//                throw ex;
//            }
//        }
//
//
//
//        @Override
//        public void onViewportTapped()
//        {
//            /*
//             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//             * when you need your vision pipeline running, but do not require a live preview on the
//             * robot controller screen. For instance, this could be useful if you wish to see the live
//             * camera preview as you are initializing your robot, but you no longer require the live
//             * preview after you have finished your initialization process; pausing the viewport does
//             * not stop running your pipeline.
//             *
//             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//             */
//
//            viewportPaused = !viewportPaused;
//
//            if(viewportPaused)
//            {
//                webcam.pauseViewport();
//            }
//            else
//            {
//                webcam.resumeViewport();
//            }
//        }
//    }
//
//    public enum RingDetectionEnum
//    {
//        UNKNOWN,
//        ZERO,
//        ONE,
//        FOUR
//    }
//
//    public enum PipelineStages
//    {
//        INPUT,
//        CONVERT2HSV,
//        MASKIMAGE,
//        BLURIMAGE,
//        OUTPUTWITHBOUNDINGRECT
//    }

    public boolean withinTolerance(double value, double tolerance, double setPoint) {
        return value >= setPoint - tolerance && value < setPoint + tolerance;
    }

}



