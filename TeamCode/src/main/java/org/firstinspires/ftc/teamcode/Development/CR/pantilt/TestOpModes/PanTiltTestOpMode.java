package org.firstinspires.ftc.teamcode.Development.CR.pantilt.TestOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Development.CR.pantilt.PanTiltContoller;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingFinderPipeline;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "CR-PanTiltTestOpMode", group = "CR-PanTilt")
public class PanTiltTestOpMode extends LinearOpMode {
    public static PIDCoefficients PAN_PID = new PIDCoefficients(-0.04, 0, 0);
    public static PIDCoefficients TILT_PID = new PIDCoefficients(-0.04, 0, 0);
    public static double DEADBANDRADIUS = 10;

    OpenCvCamera webcam;
    PanTiltContoller panTiltContoller;


    @Override
    public void runOpMode() throws InterruptedException {
        // set up the telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // set up the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RingFinderPipeline pipeline = new RingFinderPipeline(webcam, telemetry);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });

        // init the servos and home them
        panTiltContoller = new PanTiltContoller(hardwareMap);
        panTiltContoller.Initialize();

        double panAngle = panTiltContoller.getCurrentPanAngle();
        double tiltAngle = panTiltContoller.getCurrentTiltAngle();

        telemetry.addData("CurrentPanAngle", panAngle);
        telemetry.addData("CurrentTiltAngle", tiltAngle);
        telemetry.update();

        waitForStart();

        PIDFController panPID = new PIDFController(PAN_PID);
        PIDFController tiltPID = new PIDFController(TILT_PID);
        boolean usePID = true;

        while (!isStopRequested()) {

            if(usePID && pipeline.measuredArea > 0 && pipeline.centerOFTarget != null){

//                double  newPanAngle = panTiltContoller.getCurrentPanAngle(),
//                        newTiltAngle = panTiltContoller.getCurrentTiltAngle();

                double errorRadius = Math.sqrt(Math.pow(pipeline.centerOFTarget.x,2) + Math.pow(pipeline.centerOFTarget.y,2));
                if(errorRadius > DEADBANDRADIUS){
                    double panError = pipeline.centerOFTarget.x;
                    double panCorrection = panPID.update(pipeline.centerOFTarget.x);
                    double newPanAngle = panTiltContoller.getCurrentPanAngle() + panCorrection;

                    double tiltError = pipeline.centerOFTarget.y;
                    double tiltCorrection = panPID.update(pipeline.centerOFTarget.y);
                    double newTiltAngle = panTiltContoller.getCurrentTiltAngle() + tiltCorrection;

                    panTiltContoller.setPosition(newPanAngle, newTiltAngle);
                }
            }
            else if (gamepad1.x) {
                panTiltContoller.Initialize();
                usePID = !usePID;
            }
            else {
                panAngle = panTiltContoller.getCurrentPanAngle();
                tiltAngle = panTiltContoller.getCurrentTiltAngle();

                if(gamepad1.dpad_up) tiltAngle += 1;
                if(gamepad1.dpad_down) tiltAngle -= 1;
                if(gamepad1.dpad_right) panAngle += 1;
                if(gamepad1.dpad_left) panAngle -= 1;
                panTiltContoller.setPosition(panAngle, tiltAngle);
            }

            if(gamepad1.a) {
                String fileLocation =  pipeline.CaptureImage();
                telemetry.addData("file", fileLocation);

            }

            telemetry.addData("usePID", usePID);
            telemetry.addData("CurrentPanAngle", panTiltContoller.getCurrentPanAngle());
            telemetry.addData("CurrentTiltAngle", panTiltContoller.getCurrentTiltAngle());
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Rings", pipeline.RingsDetected.toString());
            if(pipeline.centerOFTarget != null) telemetry.addData("ScreenRelativeCenter", String.format("(%f, %f)", pipeline.centerOFTarget.x , pipeline.centerOFTarget.y));
            telemetry.update();

            sleep(50);

        }
   }
}
