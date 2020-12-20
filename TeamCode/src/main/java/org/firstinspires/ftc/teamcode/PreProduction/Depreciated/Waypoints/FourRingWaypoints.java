package org.firstinspires.ftc.teamcode.PreProduction.Depreciated.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;

public class FourRingWaypoints {

    public static Pose2d startPose, shootingPosition;

    public static Vector2d dropAPosition;

    public static Trajectory avoidRings, dropWobbleC, shootFromC, pickUpRing, backToShootPos, toWobble, thisThingIsInconsistent, toCWobble2, toBreakLineC;

    public static void init(SampleMecanumDrive drive) {

        shootingPosition = new Pose2d(-10, -44, Math.toRadians(-6));

        dropAPosition = new Vector2d(0, -63);

        avoidRings = drive.trajectoryBuilder(ScrimmageAutoV2.startPose)
                .strafeRight(18)
                .build();

        dropWobbleC = drive.trajectoryBuilder(avoidRings.end())
//                .splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY() - 24), 0)
                .splineToConstantHeading(dropAPosition, 0)
                .splineToConstantHeading(new Vector2d(40, -64), 0)
                .build();

        shootFromC = drive.trajectoryBuilder(dropWobbleC.end())
                .lineToLinearHeading(shootingPosition)
                .build();

        pickUpRing = drive.trajectoryBuilder(shootFromC.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX() - 38, shootingPosition.getY() + 6, 0))
                .build();

        backToShootPos = drive.trajectoryBuilder(pickUpRing.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX() + 14, shootingPosition.getY(), shootingPosition.getHeading()))
                .build();

        toWobble = drive.trajectoryBuilder(shootFromC.end())
                .lineToLinearHeading(new Pose2d(-42.25, -24.75, Math.toRadians(180)))
                .build();

        thisThingIsInconsistent = drive.trajectoryBuilder(toWobble.end())
                .forward(18)
                .build();

        toCWobble2 = drive.trajectoryBuilder(thisThingIsInconsistent.end())
                .lineToSplineHeading(new Pose2d(14, -80))
                .build();

        toBreakLineC = drive.trajectoryBuilder(toCWobble2.end())
                .splineToConstantHeading(new Vector2d(toCWobble2.end().getX() - 20, toCWobble2.end().getY()), 0)
                .splineToConstantHeading(new Vector2d(dropAPosition.getX() + 12, dropAPosition.getY()), 0)
                .build();
    }

}
