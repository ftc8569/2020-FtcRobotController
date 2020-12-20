package org.firstinspires.ftc.teamcode.PreProduction.Depreciated.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;

public class ZeroRingWaypoints {

    public static Pose2d startPose, shootingPosition;

    public static Vector2d dropAPosition;

    public static Trajectory avoidRings, dropWobbleA, shootFromA, toWobble, thisThingIsInconsistent, toAWobble2, backToBreakLine;

    public static void init(SampleMecanumDrive drive) {


        shootingPosition = new Pose2d(6, -30, Math.toRadians(-6));

        dropAPosition = new Vector2d(0, -63);

        avoidRings = drive.trajectoryBuilder(ScrimmageAutoV2.startPose)
                .strafeRight(18)
                .build();

        dropWobbleA = drive.trajectoryBuilder(avoidRings.end())
//                .splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY() - 24), 0)
                .splineToConstantHeading(dropAPosition, 0)
                .build();

        shootFromA = drive.trajectoryBuilder(dropWobbleA.end())
                .lineToLinearHeading(shootingPosition)
                .build();

        toWobble = drive.trajectoryBuilder(shootFromA.end())
                .lineToLinearHeading(new Pose2d(-18.25, -28.5, Math.toRadians(180)))
                .build();

        thisThingIsInconsistent = drive.trajectoryBuilder(toWobble.end())
                .forward(18)
                .build();

        toAWobble2 = drive.trajectoryBuilder(thisThingIsInconsistent.end())
                .lineToSplineHeading(new Pose2d(dropAPosition.getX() - 14, dropAPosition.getY() - 14, Math.toRadians(-35)))
                .build();

        backToBreakLine = drive.trajectoryBuilder(toAWobble2.end())
                .lineToConstantHeading(new Vector2d(shootingPosition.getX(), shootingPosition.getY()))
                .build();
    }

}
