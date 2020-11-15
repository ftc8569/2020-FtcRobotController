

package org.firstinspires.ftc.teamcode.Development;


import android.icu.text.Transliterator;
import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Dev: RingNumberFinderThing",  group = "Development")
@Disabled
public class RingNumberFinderThing extends LinearOpMode
{
    OpenCvWebcam phoneCam;
    RingNumberDeterminationPipeline pipeline;


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingNumberDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class RingNumberDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum NumberOfRings
        {
            ZERO,
            ONE,
            FOUR
        }

        public final double noRingUpper = 6500, oneRingUpper = 23000;




        /*
         * Working variables
         */

        Mat BGR2HSV = new Mat();



        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile NumberOfRings position = NumberOfRings.ZERO;


        void ProcessImage(Mat input)
        {
            Imgproc.cvtColor(input, BGR2HSV, Imgproc.COLOR_BGR2HSV);

        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            ProcessImage(firstFrame);


        }

        @Override
        public Mat processFrame(Mat input)
        {

            Mat copyOfInput = input;
            ProcessImage(input);
            Scalar lower = new Scalar(6, 155, 70);
            Scalar upper = new Scalar(67, 255, 255);
            Core.inRange(input, lower, upper, input);
            Imgproc.medianBlur(input, input, 11);
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            Rect rect;
            if(contours.size() > 0) {
                rect = Imgproc.boundingRect(contours.get(0));
            } else rect = null;

            Rect cropRect = new Rect(83, 1017, 642, 237);
            Mat cropImage = input.submat(cropRect);

            if(rect == null) {
                position = NumberOfRings.ZERO;
            } else {
                double area = rect.width * rect.height;
                if(area <= noRingUpper) position = NumberOfRings.ZERO;
                else if (area <= oneRingUpper) position = NumberOfRings.ONE;
                else position = NumberOfRings.FOUR;
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public NumberOfRings getAnalysis()
        {
            return position;
        }
    }
}