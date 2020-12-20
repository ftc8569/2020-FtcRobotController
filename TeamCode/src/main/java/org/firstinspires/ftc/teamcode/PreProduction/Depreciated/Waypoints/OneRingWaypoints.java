package org.firstinspires.ftc.teamcode.PreProduction.Depreciated.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageAutoV2;

public class OneRingWaypoints {

    public static Pose2d startPose, shootingPosition;

    public static Vector2d dropAPosition;

    public static Trajectory avoidRings, dropWobbleB, shootFromB, pickUpRing, backToShootPos, toWobble, thisThingIsInconsistent, toBWobble2, toBreakLineB;

    public static void init(SampleMecanumDrive drive) {


        shootingPosition = new Pose2d(-12, -30, Math.toRadians(-6));

        dropAPosition = new Vector2d(0, -63);

        avoidRings = drive.trajectoryBuilder(ScrimmageAutoV2.startPose)
                .strafeRight(18)
                .build();

        dropWobbleB = drive.trajectoryBuilder(avoidRings.end())
//                .splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY() - 24), 0)
                .splineToConstantHeading(dropAPosition, 0)
                .splineToConstantHeading(new Vector2d(12, -30), 0)
                .build();

        shootFromB = drive.trajectoryBuilder(dropWobbleB.end())
                .lineToLinearHeading(shootingPosition)
                .build();


        pickUpRing = drive.trajectoryBuilder(shootFromB.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX() - 24, shootingPosition.getY() + 6, 0))
                .build();

        backToShootPos = drive.trajectoryBuilder(pickUpRing.end())
                .lineToLinearHeading(shootingPosition)
                .build();

        toWobble = drive.trajectoryBuilder(backToShootPos.end())
                .lineToLinearHeading(new Pose2d(-28.25, -21.25, Math.toRadians(180)))
                .build();

        thisThingIsInconsistent = drive.trajectoryBuilder(toWobble.end())
                .forward(14)
                .build();

        toBWobble2 = drive.trajectoryBuilder(thisThingIsInconsistent.end())
                .lineToSplineHeading(new Pose2d(6, -56))
                .build();

        toBreakLineB = drive.trajectoryBuilder(toBWobble2.end())
                .back(12)
                .build();
    }

}
