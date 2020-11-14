package org.firstinspires.ftc.teamcode.Development.CR;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Disabled

@Config
@Autonomous(name = "Simple Auto Test", group = "Experimental")
public class SimpleAutoTestOpMode extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        CarlMecanumDrive drive = new CarlMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
    }
}
