package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.dashboard.config.Config;
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

//@Config
public class FourRingWaypoints extends AutoPaths {

    @Override
    public void init(SampleMecanumDrive drive) {
        startPose = ScrimmageAuto.startPose;
        toShoot = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(9, 14), 0)
                .splineToSplineHeading(new Pose2d(60, 14, Math.toRadians(0)), 0)
                .addDisplacementMarker(() -> drive.followTrajectory(toShootAvoidRings))
                .build();

        toShootAvoidRings = drive.trajectoryBuilder(toShoot.end())
//                .addDisplacementMarker(() -> ScrimmageAuto.sc.setPower(ScrimmageAuto.shooterPower.getPow1()))
                .splineToLinearHeading(new Pose2d(66, 42, Math.toRadians(.1)), Math.toRadians(90))
                .build();

        toDrop = drive.trajectoryBuilder(toShootAvoidRings.end())
                .splineToLinearHeading(new Pose2d(119, 23, Math.toRadians(-55)), Math.toRadians(-55))
                .build();

        toRings = drive.trajectoryBuilder(toDrop.end(), true)
                .splineToSplineHeading(new Pose2d(100, 40, 0), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(66, 40, 0), Math.toRadians(180))
                .addTemporalMarker(.9, 0, drive::cancelFollowing)
                .build();

        toRingsSlow = drive.trajectoryBuilder(toRings.get(toRings.duration() * .9))
                .splineToConstantHeading(new Vector2d(57.5, 40), Math.toRadians(180))
                .build();

        toRingsSlow2 = drive.trajectoryBuilder(toRingsSlow.end())
                .splineToConstantHeading(new Vector2d(54.5, 40), Math.toRadians(180))
//                .addDisplacementMarker(() -> drive.followTrajectory(toRingsSlow3))
                .build();

        toRingsSlow3 = drive.trajectoryBuilder(toRingsSlow2.end())
                .splineToConstantHeading(new Vector2d(51, 40), Math.toRadians(180))
                .build();




//

    }
    public void init2(SampleMecanumDrive drive) {

        toShoot2 = drive.trajectoryBuilder(toRingsSlow3.end())
                .splineToLinearHeading(new Pose2d(70, 37, Math.toRadians(0)), Math.toRadians(0))
                .build();

        toLastRing = drive.trajectoryBuilder(toShoot2.end())
                .splineToConstantHeading(new Vector2d(51, 40), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectory(toLastRingSlow))
                .build();

        toLastRingSlow = drive.trajectoryBuilder(toLastRing.end())
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(180))
                .build();

//
        toPick = drive.trajectoryBuilder(toLastRingSlow.end())
                .addTemporalMarker(0, () -> {
                    ScrimmageAuto.topMotor.setPower(-1);
                    ScrimmageAuto.bottomMotor.setPower(-1);
                })
                .addTemporalMarker(0.15, () -> {
                    ScrimmageAuto.topMotor.setPower(1);
                    ScrimmageAuto.bottomMotor.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(26.5,24, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> drive.followTrajectory(toPickSlow))
                .build();

        toPickSlow = drive.trajectoryBuilder(toPick.end())
                .splineToLinearHeading(new Pose2d(26.5, 31, Math.toRadians(90)), Math.toRadians(90))
                .build();
//
        toShoot3 = drive.trajectoryBuilder(toPickSlow.end())
                .addDisplacementMarker(() -> ScrimmageAuto.sc.setPower(ScrimmageAuto.shooterPower.getPow3()))
                .addTemporalMarker(.5, 0, () -> {
                    ScrimmageAuto.topMotor.setPower(0);
                    ScrimmageAuto.bottomMotor.setPower(0);
                        })
                .addTemporalMarker(.25, 0, () -> {
                    ScrimmageAuto.flipperMotor.setMotorEnable();
                    ScrimmageAuto.flipperMotor.setTargetPosition((int) ScrimmageAuto.armStartPos);
                })
                .splineToSplineHeading(new Pose2d(68, 37, Math.toRadians(1.5)), Math.toRadians(0))
                .build();
//
        toDrop2 = drive.trajectoryBuilder(toShoot3.end())
                .splineToLinearHeading(new Pose2d(113, 29, Math.toRadians(-55)), Math.toRadians(-55))
                .build();
//
        toLine = drive.trajectoryBuilder(toDrop2.end())
                .splineToSplineHeading(new Pose2d(84, 48, Math.toRadians(90)), Math.toRadians(90))
                .build();
//


    }
}
