package org.firstinspires.ftc.teamcode.Development.CR;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Config
@TeleOp(name = "CR-RingFinderTestOpMode", group = "Experimental")
public class RingFinderTestOpMode extends LinearOpMode
{
    public static volatile int HMIN = 60, SMIN = 100, VMIN = 50;
    public static volatile int HMAX = 120, SMAX = 250, VMAX = 250;
    public static volatile int NORINGUPPER = 200, ONERINGUPPER = 2000;
    public static volatile PipelineStages PIPELINESTAGE = PipelineStages.OUTPUTWITHBOUNDINGRECT;

    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        RingFinderPipeline pipeline = new RingFinderPipeline();
        pipeline.PipelineStageToDisplay = PIPELINESTAGE;
        webcam.setPipeline(pipeline);

        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
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

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Rings", pipeline.RingsDetected.toString());
            telemetry.addData("Display", pipeline.PipelineStageToDisplay.toString());
            telemetry.addData("HSV Metrics", pipeline.HSVSampleMetrics);
            telemetry.addData("measuredArea", pipeline.measuredArea);
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                // use the gamepad a to cycle the display image
                switch (PIPELINESTAGE){
                    case INPUT:
                        PIPELINESTAGE = PipelineStages.CONVERT2HSV;
                        break;
                    case CONVERT2HSV:
                        PIPELINESTAGE = PipelineStages.MASKIMAGE;
                        break;
                    case MASKIMAGE:
                        PIPELINESTAGE = PipelineStages.BLURIMAGE;
                        break;
                    case BLURIMAGE:
                        PIPELINESTAGE = PipelineStages.OUTPUTWITHBOUNDINGRECT;
                        break;
                    case OUTPUTWITHBOUNDINGRECT:
                    default:
                        PIPELINESTAGE = PipelineStages.INPUT;
                        break;
                }
                pipeline.PipelineStageToDisplay = PIPELINESTAGE;
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
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

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
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
                    if(measuredArea <= RingFinderTestOpMode.NORINGUPPER ) RingsDetected = RingDetectionEnum.ZERO;
                    else if (measuredArea <= RingFinderTestOpMode.ONERINGUPPER) RingsDetected = RingDetectionEnum.ONE;
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
}


