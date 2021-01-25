package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Development.CR.vision.HSVValues;
import org.firstinspires.ftc.teamcode.Development.CR.vision.PipelineStages;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingDetectionEnum;
import org.firstinspires.ftc.teamcode.Development.CR.vision.TestOpModes.RingFinderTestOpMode;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.IOException;
import java.nio.file.LinkPermission;
import java.util.ArrayList;

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
public class RingFinderPipelineButIedittedIt extends OpenCvPipeline
{
    public  RingFinderPipelineButIedittedIt (@NotNull OpenCvCamera webcamToUse, @NotNull Telemetry telemetryToUse)
    {
        super();
        webcam = webcamToUse;
        telemetry = telemetryToUse;
    }

    private FtcDashboard dashboard;
    private OpenCvCamera webcam;
    private Telemetry telemetry;
    private Mat inputImageBGR = new Mat();
    private Mat hsvImage = new Mat();
    private Mat maskImage = new Mat();
    private Mat blurImage = new Mat();
    private Mat outputImageBGR = new Mat();
    private Mat outputImageRBG = new Mat();
    private Rect BoundingRectangle = null;
    public HSVValues hsvLowerBound = new HSVValues(5,  100,  0);
    public HSVValues hsvUpperBound = new HSVValues( 25,  255,  255);

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile RingDetectionEnum RingsDetected = RingDetectionEnum.UNKNOWN;
    public volatile PipelineStages pipelineStageToDisplay = PipelineStages.INPUT;
    public volatile int measuredArea = 0;
    public volatile Point centerOFTarget = null;

    @Override
    public Mat processFrame(Mat input)
    {
        try {

            Imgproc.cvtColor(input, inputImageBGR, Imgproc.COLOR_RGB2BGR);
            inputImageBGR.copyTo(outputImageBGR);

            // Here we could possible crop the image to speed processing.  If we do, we need to do so in the init as well
            //Rect cropRect = new Rect(83, 1017, 642, 237);
            //Mat cropImage = input.submat(cropRect);
            Imgproc.cvtColor(inputImageBGR, hsvImage, Imgproc.COLOR_BGR2HSV);

            // refresh the lower/upper bounds so that this can work via FTC Dashboard
            Scalar lowerHSVBound = new Scalar(hsvLowerBound.Hue, hsvLowerBound.Saturation, hsvLowerBound.Value);
            Scalar upperHSVBound = new Scalar(hsvUpperBound.Hue, hsvUpperBound.Saturation, hsvUpperBound.Value);

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
                centerOFTarget = null;
            }
            else {
                measuredArea = BoundingRectangle.width * BoundingRectangle.height;
                double rectXCenter = BoundingRectangle.x + BoundingRectangle.width/2.0;
                double rectYCenter = BoundingRectangle.y + BoundingRectangle.height/2.0;
                double xScreenRelativeCenter = rectXCenter - input.width()/2.0;
                double yScreenRelativeCenter = rectYCenter - input.height()/2.0;
                centerOFTarget = new Point(xScreenRelativeCenter, yScreenRelativeCenter);

                if(measuredArea <= RingFinderTestOpMode.NORINGUPPER ) RingsDetected = RingDetectionEnum.ZERO;
                else if (measuredArea <= RingFinderTestOpMode.ONERINGUPPER) RingsDetected = RingDetectionEnum.ONE;
                else RingsDetected = RingDetectionEnum.FOUR;

                // Draw a simple box around the rings
                Scalar rectColor = new Scalar(0, 255, 0);
                Imgproc.rectangle(outputImageBGR, BoundingRectangle, rectColor, 4);
                int baseline[]={0};
                Size textSize = Imgproc.getTextSize(RingsDetected.toString(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,2, baseline);
                int margin = 2;
                Point textOrigin = new Point(BoundingRectangle.x + BoundingRectangle.width/2 - textSize.width/2, BoundingRectangle.y - textSize.height - margin);
                Imgproc.putText(outputImageBGR, RingsDetected.toString(), textOrigin, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rectColor, 2);
            }

            switch (pipelineStageToDisplay) {
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

                    // HSVSampleMetrics = String.format("hsv mean (%d, %d,%d), std (%d, %d,%d)", hsvMean[0] , hsvMean[1], hsvMean[2], hsvStd[0], hsvStd[1], hsvStd[2]);
                    //telemetry.addData("HSV Mean", String.format("(%d, %d,%d)", hsvMean[0] , hsvMean[1], hsvMean[2]));
                    //telemetry.addData("HSV StdDev", String.format("(%d, %d,%d)", hsvStd[0], hsvStd[1], hsvStd[2]));
                    return hsvImage;
                case OUTPUTWITHBOUNDINGRECT:
                    // we need to return an RGB image
                    Imgproc.cvtColor(outputImageBGR, outputImageRBG, Imgproc.COLOR_BGR2RGB);
                    return outputImageRBG;
                case INPUT:
                default:
                    return input;
            }
        }
        catch (Exception ex){
            throw ex;
        }
    }

    private int captureCounter = 0;
    public Mat CaptureImage() throws IOException {
        Mat image2Save;
        String fullFileName = String.format("ring-"+ pipelineStageToDisplay.toString().toLowerCase()+"-%d.png",captureCounter++);
        switch (pipelineStageToDisplay) {
            case MASKIMAGE:
                image2Save = maskImage;
                break;
            case CONVERT2HSV:
                image2Save = hsvImage;
                break;
            case BLURIMAGE:
                image2Save = blurImage;
                break;
            case OUTPUTWITHBOUNDINGRECT:
                image2Save = outputImageBGR;
                break;
            case INPUT:
            default:
                image2Save = inputImageBGR;
                break;
        }

        if(image2Save != null)        return image2Save;
        else throw new IOException("Null image to save");
    }

    public static final File VISION_FOLDER =
            new File(Environment.getExternalStorageDirectory().getPath() + "/vision/");
    public boolean saveOpenCvImageToFile(String filename, Mat mat) {

        Mat mIntermediateMat = new Mat();
        Imgproc.cvtColor(mat, mIntermediateMat, Imgproc.COLOR_BGR2RGB, 3);

        boolean mkdirs = VISION_FOLDER.mkdirs();
        File file = new File(VISION_FOLDER, filename);
        boolean savedSuccessfully = Imgcodecs.imwrite(file.toString(), mat);
        return  savedSuccessfully;
    }

}