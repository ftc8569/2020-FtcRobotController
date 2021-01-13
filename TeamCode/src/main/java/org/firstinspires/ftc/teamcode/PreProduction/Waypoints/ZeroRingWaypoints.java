package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.AutoPaths;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageAuto;
import org.opencv.core.Mat;

public class ZeroRingWaypoints extends AutoPaths {

    @Override
    public void init(SampleMecanumDrive drive) {
        startPose = ScrimmageAuto.startPose;
        toShoot = drive.trajectoryBuilder(startPose)
            .splineToSplineHeading(new Pose2d(66, 36, Math.toRadians(-4)), Math.toRadians(-6))
            .build();

        toDrop = drive.trajectoryBuilder(toShoot.end())
                .splineToSplineHeading(new Pose2d(84, 24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        toPick = drive.trajectoryBuilder(toDrop.end())
                .splineToConstantHeading(new Vector2d(84, 36), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(41.75,32.25, Math.toRadians(140)), Math.toRadians(140))
                .addDisplacementMarker(() -> drive.followTrajectory(toPickSlow))
                .build();

        toPickSlow = drive.trajectoryBuilder(toPick.end())
                .splineToLinearHeading(new Pose2d(35.75,38.25, Math.toRadians(140)), Math.toRadians(140))
                .build();

        toDrop2 = drive.trajectoryBuilder(toPickSlow.end())
                .splineToSplineHeading(new Pose2d(66.5, 20, Math.toRadians(-40)), Math.toRadians(-40))
                .build();

        toLine = drive.trajectoryBuilder(toDrop2.end())
                .splineToConstantHeading(new Vector2d(52, 32), 140)
                .splineToSplineHeading(new Pose2d(84, 48, Math.toRadians(90)), Math.toRadians(90))
                .build();
    }
}

