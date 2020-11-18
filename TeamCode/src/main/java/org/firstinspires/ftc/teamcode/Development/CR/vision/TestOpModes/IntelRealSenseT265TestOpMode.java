/*package org.firstinspires.ftc.teamcode.Development.CR.vision.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="CR-IntelRealSenseT265TestOpMode", group="CR-Vision")
public class IntelRealSenseT265TestOpMode extends LinearOpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    final int robotRadius = 9; // inches

    private static double inchesToMeters(double inches){
        return inches / 39.3700787;
    }
    private static double metersToInches(double meters){
        return meters * 39.3700787;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Transform2d transformCameraToRobot = new Transform2d(new Translation2d(216,0), new Rotation2d(Math.PI));
        Pose2d startingPose = new Pose2d(0,0, new Rotation2d(0));

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.8, hardwareMap.appContext);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        slamra.start();

        Transform2d transformToCameraStartingPose = null;
        while (transformToCameraStartingPose == null) {
            T265Camera.CameraUpdate initialUpdate = slamra.getLastReceivedCameraUpdate();
            if(initialUpdate != null){
                Pose2d cameraStartingPosition = startingPose.transformBy(transformCameraToRobot.inverse());
                transformToCameraStartingPose = cameraStartingPosition.minus(initialUpdate.pose);
            }
        }

        waitForStart();

        while(!isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up != null) {

                Pose2d pose = up.pose.transformBy(transformToCameraStartingPose).transformBy(transformCameraToRobot);
                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(pose.getTranslation().getX() / 0.0254, pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = pose.getRotation();

                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);
                packet.put("x (in)", translation.getX());
                packet.put("y (in)", translation.getY());
                packet.put("heading (deg)", rotation.getDegrees());

                dashboard.sendTelemetryPacket(packet);
            }
        }

        slamra.stop();
    }


}
*/