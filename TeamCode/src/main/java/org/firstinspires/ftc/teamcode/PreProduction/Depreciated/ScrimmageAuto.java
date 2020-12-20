package org.firstinspires.ftc.teamcode.PreProduction.Depreciated;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
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

@Disabled
@Autonomous(name = "Pre: ScrimmageAuto", group = "Pre-Production")
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

    ShooterController sc;
    public static double    grabberOpenPos   =  0.33,
            grabberClosedPos =  0,
            armStartPos = 0,
            armUpPos = -193,
            armForwardPos = -370;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, 1, 20);
    public static double pCoefficient = 15;

    public boolean open = true;

    int currentPos = 0;

    RevBlinkinLedDriver led;

    long LEDChanged = 0;

    public static RevBlinkinLedDriver.BlinkinPattern blink = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Starting initialization, please wait...");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RingFinderPipeline pipeline = new RingFinderPipeline();
        pipeline.PipelineStageToDisplay = PIPELINESTAGE;
        webcam.setPipeline(pipeline);

        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        ScrimmageTeleOp.armOffset = 0;
        ScrimmageTeleOp.headingOffset = 0;
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        LEDChanged = System.currentTimeMillis();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        grabberServo.setPosition(grabberOpenPos);

        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .025, 750, 2400);
        sc.setServo(ShooterInitializer.position.BACKWARD);




        Pose2d startPose = new Pose2d(-60, -31, Math.toRadians(-60)); //1ft, 7in, -69
        drive.setPoseEstimate(startPose);

//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .addTemporalMarker(0, () -> {
//                    flipperMotor.setTargetPosition((int) armForwardPos);
//                })
//                .addTemporalMarker(2, () -> {
//                })
//                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(10, -55, 0), 0)
//                .forward(4)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(20)
                .build();

        Trajectory traj4andahalf = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(16, -20), 0)
                .build();

//        Trajectory traj4and3quarters = drive.trajectoryBuilder(traj4andahalf.end())
//                .back(0)
//                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-70, -35, Math.toRadians(-160)), Math.toRadians(135))
                .build();

        Trajectory traj5andhalf = drive.trajectoryBuilder(traj5.end())
                .forward(15)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5andhalf.end())
                .splineToLinearHeading(new Pose2d(-10, -60, 0), 0)
                .build();

//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .forward(14)
//                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj6.end())
                .back(12)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .splineToConstantHeading(new Vector2d(12, -30), 0)
                .build();

        while(System.currentTimeMillis() - LEDChanged < 1000)
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        telemetry.addData(">", "Initialization completed.");
        telemetry.update();

        rings = RingDetectionEnum.ONE;
//        while (!isStarted() && !isStopRequested()) {
//            rings = pipeline.RingsDetected;
//        }

        waitForStart();

        if(isStopRequested()) return;

//        drive.followTrajectory(traj1);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);

//        drive.followTrajectory(traj1);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1250);
        drive.followTrajectory(traj2);
        grabberServo.setPosition(grabberClosedPos);
        sleep(500);
        flipperMotor.setTargetPosition((int) -318);
        sleep(1250);
        drive.followTrajectory(traj3);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(750);
        grabberServo.setPosition(grabberOpenPos);
        sleep(300);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj4andahalf);
        drive.turn(Math.toRadians(20));
        flipperMotor.setTargetPosition((int) ScrimmageTeleOp.armUpPos);
        sc.setPower(ScrimmageTeleOp.shooterDefaultPower);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
        sleep(1250);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.stopMotor();
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);

        flipperMotor.setTargetPosition((int) armForwardPos);
//        drive.followTrajectory(traj4and3quarters);
//        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj5andhalf);
        grabberServo.setPosition(grabberClosedPos);
        sleep(300);
        flipperMotor.setTargetPosition((int) -318);
        sleep(1250);
        drive.followTrajectory(traj6);
//        drive.followTrajectory(traj7);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(750);
        grabberServo.setPosition(grabberOpenPos);
        sleep(300);
        drive.followTrajectory(traj8);
//        drive.followTrajectory(traj9);
        ScrimmageTeleOp.armOffset = -370;
        ScrimmageTeleOp.headingOffset = drive.getLocalizer().getPoseEstimate().getHeading();

    }

    public class RingFinderPipeline extends OpenCvPipeline
    {
        private FtcDashboard dashboard;
        boolean viewportPaused;
        // these are set for 320x240 pixels at approx 30 in from camera

        public  RingFinderPipeline() {
            super();
            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25);
        }


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat hsvImage = new Mat();
        private Mat maskImage = new Mat();
        private Mat blurImage = new Mat();
        private Rect BoundingRectangle = null;
        Scalar lowerHSVBound = new Scalar(HMIN, SMIN, VMIN);
        Scalar upperHSVBound = new Scalar(HMAX, SMAX, VMAX);
        Scalar measuredLowerHSVBound = lowerHSVBound.clone();
        Scalar measuredUpperHSVBound = upperHSVBound.clone();

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingDetectionEnum RingsDetected = RingDetectionEnum.UNKNOWN;
        public volatile PipelineStages PipelineStageToDisplay = PipelineStages.OUTPUTWITHBOUNDINGRECT;
        public volatile String HSVSampleMetrics = "";
        public volatile int measuredArea = 0;
/*
        @Override
        public void init(Mat firstFrame)
        {
            Imgproc.cvtColor(firstFrame, hsvImage, Imgproc.COLOR_BGR2HSV);
            Scalar lower = new Scalar(6, 155, 70);
            Scalar upper = new Scalar(67, 255, 255);
            Core.inRange(hsvImage, lower, upper, maskImage);
            Imgproc.medianBlur(maskImage, blurImage, 11);
        }
*/

        @Override
        public Mat processFrame(Mat input)
        {
            try {
                // Here we could possible crop the image to speed processing.  If we do, we need to do so in the init as well
                //Rect cropRect = new Rect(83, 1017, 642, 237);
                //Mat cropImage = input.submat(cropRect);
                Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

                // refresh the lower/upper bounds so that this can work via FTC Dashboard
                lowerHSVBound = new Scalar(HMIN, SMIN, VMIN);
                upperHSVBound = new Scalar(HMAX, SMAX, VMAX);

                Core.inRange(hsvImage, lowerHSVBound, upperHSVBound, maskImage);
                Imgproc.medianBlur(maskImage, blurImage, 11);
                ArrayList<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(blurImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
                if(contours.size() > 0) {
                    // find the largest contour by area (this can't be assumed to be the first one in the list
                    double maxContourArea = 0D;
                    MatOfPoint maxContour = null;
                    for(int i = 0; i< contours.size(); i++) {
                        MatOfPoint contour = contours.get(i);
                        double contourArea = Imgproc.contourArea(contour);
                        if(contourArea > maxContourArea) {
                            maxContourArea = contourArea;
                            maxContour = contour;
                        }
                    }
                    // maxContour should never be null...but somehow it was and was throwing an exception...
                    if(maxContour != null) BoundingRectangle = Imgproc.boundingRect(maxContour);
                }
                else BoundingRectangle = null;

                if(BoundingRectangle == null) {
                    RingsDetected = RingDetectionEnum.ZERO;
                    measuredArea = 0;
                }
                else {
                    measuredArea = BoundingRectangle.width * BoundingRectangle.height;
                    if(measuredArea <= NORINGUPPER ) RingsDetected = RingDetectionEnum.ZERO;
                    else if (measuredArea <= ONERINGUPPER) RingsDetected = RingDetectionEnum.ONE;
                    else RingsDetected = RingDetectionEnum.FOUR;

                    // Draw a simple box around the rings
                    Scalar rectColor = new Scalar(0, 255, 0);
                    Imgproc.rectangle(input, BoundingRectangle, rectColor, 4);
                    int baseline[]={0};
                    Size textSize = Imgproc.getTextSize(RingsDetected.toString(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,2, baseline);
                    int margin = 2;
                    Point textOrigin = new Point(BoundingRectangle.x + BoundingRectangle.width/2 - textSize.width/2, BoundingRectangle.y - textSize.height - margin);
                    Imgproc.putText(input, RingsDetected.toString(), textOrigin, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
                }

                switch (PipelineStageToDisplay) {
                    case MASKIMAGE:
                        return  maskImage;
                    case BLURIMAGE:
                        return  blurImage;
                    case CONVERT2HSV:
                        // when showing hsv, put a small box on the screen and
                        // make a 100x100 square in the middle of the frame

                        int width = 100;
                        int height = 100;
                        int x = (input.width() - width)/2;
                        int y = (input.height() - height)/2;

                        Rect sampleRect = new Rect(x,y,width,height);
                        Imgproc.rectangle(hsvImage, sampleRect, new Scalar(0, 255, 0), 4);

                        // now calculate the mean and std of the sample area
                        Mat sampleImage = hsvImage.submat(sampleRect);
                        MatOfDouble mean = new MatOfDouble();
                        MatOfDouble std = new MatOfDouble();
                        Core.meanStdDev(sampleImage, mean, std);
                        double[] meanArray = mean.toArray();
                        double[] stdArray = std.toArray();
                        int[] hsvMean = new int[3];
                        int[] hsvStd = new int[3];
                        for(int i=0 ;i<3; i++){
                            hsvMean[i] = (int)meanArray[i];
                            hsvStd[i] = (int)stdArray[i];
                        }

                        HSVSampleMetrics = String.format("hsv mean (%d, %d,%d), std (%d, %d,%d)", hsvMean[0] , hsvMean[1], hsvMean[2], hsvStd[0], hsvStd[1], hsvStd[2]);
                        return hsvImage;
                    case OUTPUTWITHBOUNDINGRECT:
                    case INPUT:
                    default:
                        return input;
                }
            }
            catch (Exception ex){
                throw ex;
            }
        }



        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }

    public enum RingDetectionEnum
    {
        UNKNOWN,
        ZERO,
        ONE,
        FOUR
    }

    public enum PipelineStages
    {
        INPUT,
        CONVERT2HSV,
        MASKIMAGE,
        BLURIMAGE,
        OUTPUTWITHBOUNDINGRECT
    }
}



