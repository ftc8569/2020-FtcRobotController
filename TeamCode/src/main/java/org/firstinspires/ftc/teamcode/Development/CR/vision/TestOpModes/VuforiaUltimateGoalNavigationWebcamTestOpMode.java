package org.firstinspires.ftc.teamcode.Development.CR.vision.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Development.CR.drive.VuforiaDriveLocalizer;

import java.util.ArrayList;
import java.util.List;

import static java.lang.String.*;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="VuforiaUltimateGoalNavigationWebcamTestOpMode", group ="CR-Vision")
public class VuforiaUltimateGoalNavigationWebcamTestOpMode extends LinearOpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override public void runOpMode() {
        // Setup FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VuforiaDriveLocalizer vfDriveLocalizer = new VuforiaDriveLocalizer(hardwareMap, true);
        vfDriveLocalizer.activate();

        waitForStart();

        while (!isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            vfDriveLocalizer.update();

            packet.put("target", vfDriveLocalizer.getCurrentVisibleTargetName());

            if(vfDriveLocalizer.isTargetVisible()) {
                Canvas field = packet.fieldOverlay();
                final int robotRadius = 9; // inches
                final double maxVelocity = 10 * 12; // 10 ft/sec max for scaling

                Pose2d pose = vfDriveLocalizer.getPoseEstimate();
                Rotation2d rotation2d = new Rotation2d(pose.getHeading());
                field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
                field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
                double arrowX = rotation2d.getCos() * robotRadius, arrowY = rotation2d.getSin() * robotRadius;
                double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
                double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);

                field.setStroke("red");
                field.fillCircle(pose.getX(),pose.getY(), 0.5);
                Pose2d velocity = vfDriveLocalizer.getPoseVelocity();
                Rotation2d velRotation2d = new Rotation2d(velocity.getHeading());
                double arrowVx = velRotation2d.getCos() * robotRadius,
                        arrowVy = velRotation2d.getSin() * robotRadius;
                double velX1 = pose.getX(),
                        velY1 = pose.getY();
                double velX2 = pose.getX() + arrowVx * (velocity.getX() / maxVelocity),
                        velY2 = pose.getY() + arrowVy * (velocity.getY() / maxVelocity);
                field.strokeLine(velX1, velY1, velX2, velY2);

                packet.put("vx (in/s)", velocity.getX());
                packet.put("vy (in/s)", velocity.getY());
                packet.put("vh (deg/s)", velRotation2d.getDegrees());
                packet.put("x (in)", pose.getX());
                packet.put("y (in)", pose.getY());
                packet.put("heading (deg)", rotation2d.getDegrees());
            }
            dashboard.sendTelemetryPacket(packet);
        }

        // Disable Tracking when we are done;
        vfDriveLocalizer.deactivate();
    }
}
