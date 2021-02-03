package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.AutoPaths;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageAuto;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageTeleOp;
import org.opencv.core.Mat;

@Config
public class ZeroRingWaypoints extends AutoPaths {

    @Override
    public void init(SampleMecanumDrive drive) {
        startPose = ScrimmageAuto.startPose;
        toShoot = drive.trajectoryBuilder(startPose)
            .splineToConstantHeading(new Vector2d(65, 36), Math.toRadians(0.0))
            .build();

        toDrop = drive.trajectoryBuilder(toShoot.end())
                .splineToSplineHeading(new Pose2d(76, 24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        toPick = drive.trajectoryBuilder(toDrop.end())
                .splineToConstantHeading(new Vector2d(84, 36), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(41.75,32.25, Math.toRadians(145)), Math.toRadians(140))
                .addDisplacementMarker(() -> drive.followTrajectory(toPickSlow))
                .build();

        toPickSlow = drive.trajectoryBuilder(toPick.end())
                .splineToLinearHeading(new Pose2d(33.25,30, Math.toRadians(145)), Math.toRadians(142))
                .build();

        toDrop2 = drive.trajectoryBuilder(toPickSlow.end())
                .splineToSplineHeading(new Pose2d(72, 17, Math.toRadians(-40)), Math.toRadians(-40))
                .build();

        toLine = drive.trajectoryBuilder(toDrop2.end())
                .splineToConstantHeading(new Vector2d(52, 17), 140)
                .splineToSplineHeading(new Pose2d(84, 48, Math.toRadians(90)), Math.toRadians(90))
                .build();
    }
}

