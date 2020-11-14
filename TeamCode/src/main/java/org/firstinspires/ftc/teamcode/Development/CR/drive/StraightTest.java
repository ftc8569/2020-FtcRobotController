package org.firstinspires.ftc.teamcode.Development.CR.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Development.CR.CarlMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Disabled

@Config
@Autonomous(name="CR3 Straight Test", group = "Experimental")
public class StraightTest extends LinearOpMode {
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
