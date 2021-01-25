package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterInitializer;
import org.firstinspires.ftc.teamcode.Development.ET.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Development.ET.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PreProduction.ScrimmageTeleOp;

//uses a standard gecko wheel
@Config
@Autonomous
public class PowerShotsAutoWithNotAGeckoWheel extends LinearOpMode {

    SampleMecanumDrive drive;


    public static double shotPower1 = -.6125, shotPower2 = -.6125, shotPower3 = -.6125;

    ShooterController sc;

    public static double shot1y = -25.5, shotdelta1 = 6.75, shotdelta2 = 15.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.maxVel /= 2;
        DriveConstants.maxAccel /= 2;
        DriveConstants.maxAngleVel /= 2;
        DriveConstants.maxAngleAccel /= 2;

        SampleMecanumDrive.FOLLOWER_POSITION_TOLERANCE = .1;
        SampleMecanumDrive.FOLLOWER_HEADING_TOLERANCE = Math.toRadians(.25);
        SampleMecanumDrive.FOLLOWER_TIMEOUT = 1;

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory toShot1 = drive.trajectoryBuilder(new Pose2d())
                        .lineToConstantHeading(new Vector2d(0, shot1y))
                        .build(),
                toShot2 = drive.trajectoryBuilder(toShot1.end())
                        .lineToConstantHeading(new Vector2d(0, shot1y - shotdelta1))
                        .build(),
                toShot3 = drive.trajectoryBuilder(toShot2.end())
                        .lineToConstantHeading(new Vector2d(0, shot1y - shotdelta2))
                        .build();

        sc = new ShooterController(DonutShooter2000Controller.class, hardwareMap, .006, 750, 2400);
        sc.setServo(ShooterInitializer.position.BACKWARD);
        sc.setPIDF(ScrimmageTeleOp.shooterCoeffs);


        telemetry.addData(">", "Init Completed");
        telemetry.update();

        waitForStart();

        drive.followTrajectory(toShot1);


        shooting1:
        {
            //spins up the shooter so that it is prepared to shoot
                    sc.setPower(shotPower1);

            //sleep to give the shooter time to spin up
                    sleep(1250);
//            long time = System.currentTimeMillis();
            while (!sc.canShoot()) ;

            //shooting code. Basically setting servo forwards and back repeatedly
            sc.setServo(ShooterInitializer.position.FORWARD);
            sleep(350);
            sc.setServo(ShooterInitializer.position.BACKWARD);
            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow2());
//            sleep(350);
//            while (!sc.canShoot()) ;


//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
//            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow3());
//            sleep(350);
//            while (!sc.canShoot()) ;
//
//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
            sleep(100);

            sc.setPower(shotPower2);

            //stops shooter motor and intake if it is moving
//            sc.stopMotor();

        }

        drive.followTrajectory(toShot2);

        shooting2:
        {
            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow1());

            //sleep to give the shooter time to spin up
            sleep(500);
//            long time = System.currentTimeMillis();
            while (!sc.canShoot()) ;

            //shooting code. Basically setting servo forwards and back repeatedly
            sc.setServo(ShooterInitializer.position.FORWARD);
            sleep(350);
            sc.setServo(ShooterInitializer.position.BACKWARD);
            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow2());
//            sleep(350);
//            while (!sc.canShoot()) ;


//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
//            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow3());
//            sleep(350);
//            while (!sc.canShoot()) ;
//
//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
            sleep(100);

            sc.setPower(shotPower3);
            //stops shooter motor and intake if it is moving
//            sc.stopMotor();

        }

        drive.followTrajectory(toShot3);

        shooting3:
        {
            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow1());

            //sleep to give the shooter time to spin up
            sleep(500);
//            long time = System.currentTimeMillis();
            while (!sc.canShoot()) ;

            //shooting code. Basically setting servo forwards and back repeatedly
            sc.setServo(ShooterInitializer.position.FORWARD);
            sleep(350);
            sc.setServo(ShooterInitializer.position.BACKWARD);
            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow2());
//            sleep(350);
//            while (!sc.canShoot()) ;


//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
//            //spins up the shooter so that it is prepared to shoot
//            sc.setPower(shooterPower.getPow3());
//            sleep(350);
//            while (!sc.canShoot()) ;
//
//            sc.setServo(ShooterInitializer.position.FORWARD);
//            sleep(350);
//            sc.setServo(ShooterInitializer.position.BACKWARD);
            sleep(200);

            //stops shooter motor and intake if it is moving
            sc.stopMotor();

        }


    }
}
