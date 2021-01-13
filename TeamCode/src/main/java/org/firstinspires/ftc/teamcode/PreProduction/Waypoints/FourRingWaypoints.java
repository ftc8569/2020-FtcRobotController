package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.AutoPaths;
import org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageAuto;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageTeleOp;

import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAccel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAngleAccel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxAngleVel;
import static org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants.maxVel;
import static org.firstinspires.ftc.teamcode.PreProduction.ScrimmageAuto.slowFactor;

public class FourRingWaypoints extends AutoPaths {

    @Override
    public void init(SampleMecanumDrive drive) {
        startPose = ScrimmageAuto.startPose;
        toShoot = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(9, 14), 0)
                .splineToSplineHeading(new Pose2d(60, 14, 0), 0)
                .addDisplacementMarker(() -> drive.followTrajectory(toShootAvoidRings))
                .build();

        toShootAvoidRings = drive.trajectoryBuilder(toShoot.end())
                .addDisplacementMarker(() -> ScrimmageAuto.sc.setPower(ScrimmageAuto.shooterPower.getPow1()))
                .splineToLinearHeading(new Pose2d(66, 42, Math.toRadians(-4)), Math.toRadians(90))
                .build();

        toDrop = drive.trajectoryBuilder(toShootAvoidRings.end())
                .splineToLinearHeading(new Pose2d(119, 23, Math.toRadians(-55)), Math.toRadians(-55))
                .build();

        toRings = drive.trajectoryBuilder(toDrop.end(), true)
                .splineToSplineHeading(new Pose2d(100, 40, 0), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(66, 40, 0), Math.toRadians(180))
                .addTemporalMarker(.9, 0, drive::cancelFollowing)
//                .addDisplacementMarker(() -> drive.followTrajectory(dropIntake))
                .build();
        //              63
////        dropIntake = drive.trajectoryBuilder(toRings.end())
////                .lineToConstantHeading(new Vector2d(63, 40))
////                .addTemporalMarker(.5, 0, drive::cancelFollowing)
////                .build();
//
//        dropIntake2 = drive.trajectoryBuilder(dropIntake.get(dropIntake.duration() / 2))
//                .splineToSplineHeading(new Pose2d(66, 40, 0), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toRingsSlow))
//                .build();

//        DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(
//                maxVel / slowFactor, maxAccel / slowFactor/*62.7698851567038, 62.7698851567038*/, 0.0, maxAngleVel / slowFactor,
//                maxAngleAccel / slowFactor, 0.0
//        );

        toRingsSlow = drive.trajectoryBuilder(toRings.get(toRings.duration() * .9))
                .splineToConstantHeading(new Vector2d(57.5, 40), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toRingsSlow2))
                .build();

        toRingsSlow2 = drive.trajectoryBuilder(toRingsSlow.end())
                .splineToConstantHeading(new Vector2d(54.5, 40), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toRingsSlow3))
                .build();

        toRingsSlow3 = drive.trajectoryBuilder(toRingsSlow2.end())
                .splineToConstantHeading(new Vector2d(51, 40), Math.toRadians(180))
                .build();

//        DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(
//                maxVel * slowFactor, maxAccel * slowFactor/*62.7698851567038, 62.7698851567038*/, 0.0, maxAngleVel * slowFactor,
//                maxAngleAccel * slowFactor, 0.0
//        );

        toShoot2 = drive.trajectoryBuilder(toRingsSlow3.end())
                .splineToSplineHeading(new Pose2d(68, 37, Math.toRadians(-2.5)), Math.toRadians(0))
                .build();

        toLastRing = drive.trajectoryBuilder(toShoot2.end())
                .splineToConstantHeading(new Vector2d(51, 40), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectory(toLastRingSlow))
                .build();
//
        toLastRingSlow = drive.trajectoryBuilder(toLastRing.end())
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toPick))
                .build();
//
//        toPick = drive.trajectoryBuilder(toLastRingSlow.end())
//                .splineToSplineHeading(new Pose2d(40.75,35.75, Math.toRadians(140)), Math.toRadians(140))
//                .addDisplacementMarker(() -> drive.followTrajectory(toPickSlow))
//                .build();
//
//        toPickSlow = drive.trajectoryBuilder(toPick.end())
//                .splineToLinearHeading(new Pose2d(33.25,43.25, Math.toRadians(140)), Math.toRadians(140))
//                .build();
//
        toShoot3 = drive.trajectoryBuilder(toLastRingSlow.end())
                .addDisplacementMarker(() -> ScrimmageAuto.sc.setPower(ScrimmageAuto.shooterPower.getPow3()))
                .addTemporalMarker(.5, 0, () -> {
                    ScrimmageAuto.topMotor.setPower(0);
                    ScrimmageAuto.bottomMotor.setPower(0);
                        })
                .splineToSplineHeading(new Pose2d(68, 37, Math.toRadians(-2.5)), Math.toRadians(0))
                .build();
//
//        toDrop2 = drive.trajectoryBuilder(toShoot3.end())
//                .splineToLinearHeading(new Pose2d(113, 29, Math.toRadians(-55)), Math.toRadians(-55))
//                .build();
//
        toLine = drive.trajectoryBuilder(toShoot3.end())
                .splineToSplineHeading(new Pose2d(84, 48, Math.toRadians(90)), Math.toRadians(90))
                .build();
//


    }
}
