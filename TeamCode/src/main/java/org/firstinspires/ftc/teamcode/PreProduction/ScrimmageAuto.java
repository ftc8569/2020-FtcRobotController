package org.firstinspires.ftc.teamcode.PreProduction;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;

@Autonomous(name = "Pre: ScrimmageAuto", group = "Pre-Production")
public class ScrimmageAuto extends LinearOpMode {
    public Servo grabberServo;
    public DcMotorEx flipperMotor;
    long lastPressed = 0;

    public static double    grabberOpenPos   =  0.33,
            grabberClosedPos =  0,
            armStartPos = 0,
            armUpPos = -183,
            armForwardPos = -368;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, .5, 10);
    public static double pCoefficient = 10;

    public boolean open = true;

    int currentPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        flipperMotor = this.hardwareMap.get(DcMotorEx.class, "flipperMotor");
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperMotor.setTargetPosition(currentPos);
        flipperMotor.setPower(.125);
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipperMotor.setVelocityPIDFCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);
        flipperMotor.setPositionPIDFCoefficients(pCoefficient);

        grabberServo = this.hardwareMap.servo.get("grabberServo");
        this.grabberServo.setDirection(Servo.Direction.REVERSE);
        grabberServo.setPosition(grabberOpenPos);


        Pose2d startPose = new Pose2d(-60, -31, Math.toRadians(-60)); //1ft, 7in, -69
        drive.setPoseEstimate(startPose);

//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .addTemporalMarker(0, () -> {
//                    flipperMotor.setTargetPosition((int) armForwardPos);
//                })
//                .addTemporalMarker(2, () -> {
//                })
//                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(10, -56, 0), 0)
                .forward(18)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(50)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-46, -38, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .splineToLinearHeading(new Pose2d(10, -56, 0), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

//        drive.followTrajectory(traj1);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1500);
        drive.followTrajectory(traj2);
        grabberServo.setPosition(grabberClosedPos);
        sleep(200);
        flipperMotor.setTargetPosition((int) -318);
        sleep(1500);
        drive.followTrajectory(traj3);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1000);
        grabberServo.setPosition(grabberOpenPos);
        sleep(500);
        drive.followTrajectory(traj4);
        grabberServo.setPosition(grabberClosedPos);
        sleep(500);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        grabberServo.setPosition(grabberOpenPos);
        sleep(500);

    }
}
