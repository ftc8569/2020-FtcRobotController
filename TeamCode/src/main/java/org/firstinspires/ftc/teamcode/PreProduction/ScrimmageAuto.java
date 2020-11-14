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

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;

@Autonomous(name = "Pre: ScrimmageAuto", group = "Pre-Production")
public class ScrimmageAuto extends LinearOpMode {
    public Servo grabberServo;
    public DcMotorEx flipperMotor;
    long lastPressed = 0;

    ShooterController sc;
    public static double    grabberOpenPos   =  0.33,
            grabberClosedPos =  0,
            armStartPos = ScrimmageTeleOp.armStartPos,
            armUpPos = ScrimmageTeleOp.armUpPos,
            armForwardPos = ScrimmageTeleOp.armForwardPos;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(15, 10, 1, 20);
    public static double pCoefficient = 15;

    public boolean open = true;

    int currentPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Starting initialization, please wait...");
        telemetry.update();
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

        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .025, 750, 2400);
        sc.setServo(ShooterInitializer.position.BACKWARD);



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
                .splineToSplineHeading(new Pose2d(10, -60, 0), 0)
                .forward(4)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(20)
                .build();

        Trajectory traj4andahalf = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(15, -20), 0)
                .build();

//        Trajectory traj4and3quarters = drive.trajectoryBuilder(traj4andahalf.end())
//                .back(0)
//                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(-70, -35, Math.toRadians(-160)), Math.toRadians(135))
                .build();

        Trajectory traj5andhalf = drive.trajectoryBuilder(traj5.end())
                .forward(10)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5andhalf.end())
                .splineToLinearHeading(new Pose2d(-10, -65, 0), 0)
                .build();

//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .forward(14)
//                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj6.end())
                .back(12)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .splineToConstantHeading(new Vector2d(12, -30), 0)
                .build();

        telemetry.addData(">", "Initialization completed.");
        telemetry.update();

        if(isStopRequested()) return;

        waitForStart();

        if(isStopRequested()) return;

//        drive.followTrajectory(traj1);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1500);
        drive.followTrajectory(traj2);
        grabberServo.setPosition(grabberClosedPos);
        sleep(500);
        flipperMotor.setTargetPosition((int) -318);
        sleep(1500);
        drive.followTrajectory(traj3);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1000);
        grabberServo.setPosition(grabberOpenPos);
        sleep(500);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj4andahalf);
        drive.turn(Math.toRadians(10));
        flipperMotor.setTargetPosition((int) ScrimmageTeleOp.armUpPos);
        sc.setPower(ScrimmageTeleOp.shooterDefaultPower);
        sleep(1500);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);
        sc.stopMotor();
        flipperMotor.setTargetPosition((int) armForwardPos);
//        drive.followTrajectory(traj4and3quarters);
//        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj5andhalf);
        grabberServo.setPosition(grabberClosedPos);
        sleep(500);
        flipperMotor.setTargetPosition((int) -318);
        sleep(1500);
        drive.followTrajectory(traj6);
//        drive.followTrajectory(traj7);
        flipperMotor.setTargetPosition((int) armForwardPos);
        sleep(1000);
        grabberServo.setPosition(grabberOpenPos);
        sleep(500);
        drive.followTrajectory(traj8);
//        drive.followTrajectory(traj9);
        ScrimmageTeleOp.armOffset = flipperMotor.getTargetPosition();
        ScrimmageTeleOp.headingOffset = drive.getLocalizer().getPoseEstimate().getHeading();

    }


}
