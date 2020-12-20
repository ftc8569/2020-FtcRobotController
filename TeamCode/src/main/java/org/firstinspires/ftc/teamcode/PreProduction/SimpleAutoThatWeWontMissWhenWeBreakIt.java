package org.firstinspires.ftc.teamcode.PreProduction;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.DonutShooter2000Controller;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShotPowers;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.Depreciated.ScrimmageTeleOp;

@Autonomous(name = "Pre: SimpleAuto", group = "Pre-Production", preselectTeleOp = "Pre: ScrimmageTeleOpWithAutomation")
public class SimpleAutoThatWeWontMissWhenWeBreakIt extends LinearOpMode {

    ShooterController sc;



    DcMotorEx topMotor, bottomMotor;

    public static Pose2d startPose;

    //we need to offset the power slightly because we are further back than we shoot from in teleop
    double PowerOffset = 0;

    public static ShotPowers pows = new ShotPowers(-.71, -.7025, -.69); // powers for straight on shooting.

    RevBlinkinLedDriver led;

    long LEDChanged = 0;





    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Starting initialization, please wait...");
        telemetry.update();

        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        LEDChanged = System.currentTimeMillis();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        startPose = new Pose2d(0, 10, Math.toRadians(-0)); //1ft, 7in, -69
        drive.setPoseEstimate(startPose);

        Trajectory toShootPos = drive.trajectoryBuilder(startPose)
                .forward(56)
                .build();

        Trajectory breakLine = drive.trajectoryBuilder(toShootPos.end())
                .splineToLinearHeading(new Pose2d(toShootPos.end().getX() + 16, toShootPos.end().getY(), Math.toRadians(90)), 0)
                .build();


        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .009, 1000, 2400);
        sc.setServo(ShooterInitializer.position.BACKWARD);


        while(System.currentTimeMillis() - LEDChanged < 1000);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        telemetry.addData(">", "Initialization completed.");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(toShootPos);
        drive.turn(Math.toRadians(-23));

        //spins up the shooter so that it is prepared to shoot
        sc.setPower(ScrimmageTeleOp.pows.pow1 - PowerOffset);


        //sleep to give the shooter time to spin up
        sleep(1000);
        while (!sc.canShoot()) ;

        //shooting code. Basically setting servo forwards and back repeatedly
        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);

        sc.setPower(ScrimmageTeleOp.pows.pow2 - PowerOffset);
        sleep(1000);
        while (!sc.canShoot());

        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(250);

        sc.setPower(ScrimmageTeleOp.pows.pow3 - PowerOffset);
        sleep(1000);
        while (!sc.canShoot());

        sc.setServo(ShooterInitializer.position.FORWARD);
        sleep(250);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sleep(100);

        sc.stopMotor();

        drive.followTrajectory(breakLine);
//        drive.turn(Math.toRadians(110));


    }

}
