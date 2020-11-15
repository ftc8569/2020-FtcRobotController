package org.firstinspires.ftc.teamcode.Development.CR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Development.CR.drive.CarlMecanumDrive;
import org.firstinspires.ftc.teamcode.Development.CR.vision.RingFinderPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@Config
@Autonomous(name = "CR-SimpleAutoTestOpMode", group = "CR-Main")
public class SimpleAutoTestOpMode extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        CarlMecanumDrive drive = new CarlMecanumDrive(hardwareMap);


        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        TrajectoryBuilder targetZoneTrajectoryBuilder = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(36,-12), 0);
        Trajectory targetZoneTrajectory, returnToShootingLineTrajectory;

        switch (pipeline.RingsDetected)
        {
            case FOUR:
                // Target Zone C
                targetZoneTrajectoryBuilder.forward(84);
                targetZoneTrajectory = targetZoneTrajectoryBuilder.build();
                returnToShootingLineTrajectory = drive.trajectoryBuilder(targetZoneTrajectory.end())
                        .back(48).build();
                break;
            case ONE:
                // Target Zone B
                targetZoneTrajectoryBuilder.forward(60)
                        .strafeLeft(24);
                targetZoneTrajectory = targetZoneTrajectoryBuilder.build();
                returnToShootingLineTrajectory = drive.trajectoryBuilder(targetZoneTrajectory.end())
                        .back(24).build();
                break;
            case ZERO:
            case UNKNOWN:
            default:
                // Target Zone A
                targetZoneTrajectoryBuilder.forward(36);
                targetZoneTrajectory = targetZoneTrajectoryBuilder.build();
                returnToShootingLineTrajectory = drive.trajectoryBuilder(targetZoneTrajectory.end())
                        .back(12).build();
                break;
        }

        drive.followTrajectory(targetZoneTrajectory);
        sleep(5000); // wait there 5 seconds
        drive.followTrajectory(returnToShootingLineTrajectory);


    }
}
