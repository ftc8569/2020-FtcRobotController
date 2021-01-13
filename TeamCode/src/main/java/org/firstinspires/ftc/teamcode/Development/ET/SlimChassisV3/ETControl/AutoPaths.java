package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;

public abstract class AutoPaths {

    public abstract void init(SampleMecanumDrive drive);

    public Pose2d startPose;

    public Trajectory toShoot, toShootAvoidRings, toRings, dropIntake, dropIntake2, toRingsSlow, toRingsSlow2, toRingsSlow3, toLastRing, toLastRingSlow, toShoot3, toShoot2, toDrop, toPick, toPickSlow, toDrop2, toLine;

}
