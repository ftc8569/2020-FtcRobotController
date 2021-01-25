package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.AutoPaths;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageAuto;
import org.opencv.core.Mat;

//@Config
public class OneRingWaypoints extends AutoPaths {

    @Override
    public void init(SampleMecanumDrive drive) {
        startPose = ScrimmageAuto.startPose;
        toShoot = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(9, 14), 0)
                .splineToSplineHeading(new Pose2d(60, 14, Math.toRadians(1)), 0)
                .addDisplacementMarker(() -> drive.followTrajectory(toShootAvoidRings))
                .build();

        toShootAvoidRings = drive.trajectoryBuilder(toShoot.end())
                .splineToLinearHeading(new Pose2d(66, 42, Math.toRadians(0)), Math.toRadians(90))
                .build();

        toDrop = drive.trajectoryBuilder(toShootAvoidRings.end())
                .splineToLinearHeading(new Pose2d(90, 41, 0), 0)
                .build();

        toRings = drive.trajectoryBuilder(toDrop.end())
                .splineToConstantHeading(new Vector2d(63, 36), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toRingsSlow))
                .addTemporalMarker(.9, 0, drive::cancelFollowing)
                .build();

        toRingsSlow = drive.trajectoryBuilder(toRings.get(toRings.duration() * .9))
                .splineToConstantHeading(new Vector2d(53, 41
                ), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toPick))
                .build();

        toPick = drive.trajectoryBuilder(toRingsSlow.end())
                .splineToSplineHeading(new Pose2d(42.5,38, Math.toRadians(142.5)), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectory(toPickSlow))
                .build();

        toPickSlow = drive.trajectoryBuilder(toPick.end())
                .splineToLinearHeading(new Pose2d(32,43, Math.toRadians(142.5)), Math.toRadians(140))
                .build();

        toDrop2 = drive.trajectoryBuilder(toPickSlow.end())
                .splineToSplineHeading(new Pose2d(86, 28, 0),0)
                .build();

        toShoot2 = drive.trajectoryBuilder(toDrop2.end())
                .splineToConstantHeading(new Vector2d(71, 33), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(68, 42, Math.toRadians(2.5)), Math.toRadians(90))
                .build();

        toLine = drive.trajectoryBuilder(toShoot2.end())
                .splineToSplineHeading(new Pose2d(84, 48, Math.toRadians(90)), Math.toRadians(0))
                .build();

    }

}
