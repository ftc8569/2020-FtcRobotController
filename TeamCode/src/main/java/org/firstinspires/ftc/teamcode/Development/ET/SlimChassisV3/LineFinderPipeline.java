package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import android.os.Environment;
import android.os.PowerManager;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.util.ArrayList;
import java.util.List;

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
public class LineFinderPipeline extends OpenCvPipeline {
    public LineFinderPipeline(@NotNull OpenCvCamera webcamToUse, @NotNull Telemetry telemetryToUse) {
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
    private Mat cannyImage = new Mat();
    private Mat outputImageBGR = new Mat();
    private Mat outputImageRBG = new Mat();
    private Rect BoundingRectangle = null;
    public HSVValues hsvLowerBound = new HSVValues(75, 25, 25);
    public HSVValues hsvUpperBound = new HSVValues(179, 196, 255);

    public double   horizontal_line_count = 0,
                    horizontal_rho_total = 0,
                    horizontal_theta_total = 0,
                    vertical_line_count = 0,
                    vertical_rho_total = 0,
                    vertical_theta_total = 0;

    public ArrayList<Double> x0 = new ArrayList(), y0 = new ArrayList(), x1 = new ArrayList(), y1 = new ArrayList(), x2 = new ArrayList(), y2 = new ArrayList();

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile double horizontal_rho = 0, horizontal_theta = 0, vertical_rho = 0, vertical_theta = 0;
    public volatile PipelineStages pipelineStageToDisplay = PipelineStages.INPUT;
    public volatile int measuredArea = 0;
    public volatile Point centerOFTarget = null;

    @Override
    public Mat processFrame(Mat input) {
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

            Imgproc.Canny(cannyImage, maskImage, 0, 1);

            Imgproc.HoughLines(outputImageBGR, cannyImage, 1, Math.PI/180, 50);

            for (int x = 0; x < outputImageBGR.rows(); x++) {
                double rho = outputImageBGR.get(x, 0)[0],
                        theta = outputImageBGR.get(x, 0)[1];

                double horizontal_line_theta = 90.0/180 * Math.PI;
                double horizontal_line_threshold = 10.0/180 * Math.PI;

                if(Math.abs(theta - horizontal_line_theta) < horizontal_line_threshold) {
                    horizontal_rho_total += rho;
                    horizontal_theta_total += theta;
                    horizontal_line_count += 1;
                } else {
                    vertical_rho_total += rho;
                    vertical_theta_total += theta;
                    vertical_line_count += 1;
                }
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a*rho, y0 = b*rho;
                this.x0.add(x0);
                this.y0.add(y0);


                x1.add(x0 + 1000*(-b));
                y1.add(y0 + 1000*(a));
                x2.add(x0 - 1000*(-b));
                y2.add(y0 - 1000*(a));
            }

            horizontal_rho = horizontal_rho_total / horizontal_line_count;
            horizontal_theta = horizontal_theta_total / horizontal_line_count;

            vertical_rho = vertical_rho_total / vertical_line_count;
            vertical_theta = vertical_theta_total / vertical_line_count;

            for(int j = 0; j < x0.size(); j++) {
                Imgproc.line(outputImageBGR, new Point(x1.get(j), y1.get(j)), new Point(x2.get(j), y2.get(j)), new Scalar(0, 0, 255), 2);
            }

            switch (pipelineStageToDisplay) {
                case MASKIMAGE:
                    return maskImage;
                case BLURIMAGE:
                    return blurImage;
                case CONVERT2HSV:

                    return hsvImage;
                case OUTPUTWITHLINES:
                    // we need to return an RGB image
                    Imgproc.cvtColor(outputImageBGR, outputImageRBG, Imgproc.COLOR_BGR2RGB);
                    return outputImageRBG;
                case INPUT:
                default:
                    return input;
            }
        } catch (Exception ex) {
            throw ex;
        }
    }

    private int captureCounter = 0;

    public Mat CaptureImage() throws IOException {
        Mat image2Save;
        String fullFileName = String.format("ring-" + pipelineStageToDisplay.toString().toLowerCase() + "-%d.png", captureCounter++);
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
            case OUTPUTWITHLINES:
                image2Save = outputImageBGR;
                break;
            case INPUT:
            default:
                image2Save = inputImageBGR;
                break;
        }

        if (image2Save != null) return image2Save;
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
        return savedSuccessfully;
    }

    public enum PipelineStages
    {
        INPUT,
        CONVERT2HSV,
        MASKIMAGE,
        BLURIMAGE,
        CANNYIMAGE,
        OUTPUTWITHLINES
    }
}


