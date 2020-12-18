package org.firstinspires.ftc.teamcode.Development.CR.drive.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Development.CR.drive.CarlDriveConstants;
import org.firstinspires.ftc.teamcode.Development.CR.drive.CarlMecanumDrive;
import org.firstinspires.ftc.teamcode.Development.CR.drive.OdometryPod;
import org.firstinspires.ftc.teamcode.Development.CR.drive.ThreeWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Development.CR.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.Development.CR.util.Encoder;

@TeleOp(name="CR1 RevEncoderTestOpMode")
public class RevEncoderTestOpMode extends LinearOpMode {

    public enum TestMode {
        JOYSTICKS,
        DPAD,
        TRAJECTORY_LINE,
        TRAJECTORY_SQUARE,
        RETURNING_HOME
    }

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private ThreeWheelTrackingLocalizer threeWheelTrackingLocalizer;
    private TwoWheelTrackingLocalizer twoWheelLeftFrontTrackingLocalizer;
    private TwoWheelTrackingLocalizer twoWheelRightFrontTrackingLocalizer;
    private MecanumDrive.MecanumLocalizer mecanumLocalizer;
    private TestMode testMode =  TestMode.JOYSTICKS;
    private TestMode nextTestMode = TestMode.DPAD;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CarlMecanumDrive drive = new CarlMecanumDrive(hardwareMap);

        // dimensions are relative to robot coordinates and heading is in radians
        OdometryPod leftPod = new OdometryPod(hardwareMap,"leftOdoEncoder", new Pose2d(0.8, 7.5, 0.0 )),
                rightPod = new OdometryPod(hardwareMap,"rightOdoEncoder", new Pose2d(0.8, -7.5, 0.0 ), Encoder.Direction.REVERSE),
                frontPod = new OdometryPod(hardwareMap,"frontOdoEncoder", new Pose2d(8.5, 0.8, Math.toRadians(90.0)), Encoder.Direction.REVERSE);

        threeWheelTrackingLocalizer = new ThreeWheelTrackingLocalizer(leftPod, rightPod,frontPod);
        twoWheelLeftFrontTrackingLocalizer = new TwoWheelTrackingLocalizer(leftPod, frontPod, drive);
        twoWheelRightFrontTrackingLocalizer = new TwoWheelTrackingLocalizer(rightPod, frontPod, drive);
        mecanumLocalizer = drive.mecanumLocalizer;

        waitForStart();

        while (!isStopRequested()) {
            double velFactor = 0.1;
            Pose2d velocity;

            if(gamepad1.a) {
                while(gamepad1.a) sleep(100); // wait for release to debounce
                TestMode oldTestMode = testMode;
                testMode = nextTestMode;
                switch (oldTestMode) {
                    case JOYSTICKS:
                        nextTestMode = TestMode.DPAD;
                        break;
                    case DPAD:
                        nextTestMode = TestMode.TRAJECTORY_LINE;
                        break;
                    case TRAJECTORY_LINE:
                        nextTestMode = TestMode.TRAJECTORY_SQUARE;
                        break;
                    case TRAJECTORY_SQUARE:
                        nextTestMode = TestMode.JOYSTICKS;
                        break;
                    default:
                        nextTestMode = TestMode.JOYSTICKS;
                        break;
                }
            }

            switch (testMode) {
                case JOYSTICKS:
                {
                    Pose2d baseVel = new Pose2d(
                            -gamepad1.left_stick_y * velFactor,
                            -gamepad1.left_stick_x * velFactor,
                            -gamepad1.right_stick_x * velFactor
                    );

                    if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                        // re-normalize the powers according to the weights
                        double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                                + VY_WEIGHT * Math.abs(baseVel.getY())
                                + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                        velocity = new Pose2d(
                                VX_WEIGHT * baseVel.getX(),
                                VY_WEIGHT * baseVel.getY(),
                                OMEGA_WEIGHT * baseVel.getHeading()
                        ).div(denom);
                    } else {
                        velocity = baseVel;
                    }
                    drive.setDrivePower(velocity);
                    drive.update();
                    break;
                }
                case DPAD:
                {
                    if(gamepad1.dpad_up) velocity = new Pose2d(velFactor, 0.0, 0.0);
                    else if(gamepad1.dpad_down) velocity = new Pose2d(-velFactor, 0.0, 0.0);
                    else if(gamepad1.dpad_right) velocity = new Pose2d(0.0, -velFactor, 0.0);
                    else if(gamepad1.dpad_down) velocity = new Pose2d(0.0, velFactor, 0.0);
                    else velocity = new Pose2d(0.0, 0.0, 0.0);
                    drive.setDrivePower(velocity);
                    drive.update();
                    break;
                }
                case TRAJECTORY_LINE:
                {
                    Vector2d topOfLine = new Vector2d(12.0, 0.0);
                    Vector2d bottomOfLine = new Vector2d(-12.0, 0.0);
                    Vector2d home = new Vector2d(0.0, 0.0);

                    // BASECONTRAINTS:  public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(34.4, 34.4, 0.0, Math.toRadians(260), Math.toRadians(260), 0.0);
                    //DriveConstraints driveConstraints = new DriveConstraints(10,5,0, Math.toRadians(90), Math.toRadians(90),0 );
                    DriveConstraints driveConstraints = CarlDriveConstants.BASE_CONSTRAINTS;

                    if(gamepad1.dpad_up) {
                        while (gamepad1.dpad_up) sleep(100); // wait for release to debounce
                        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(topOfLine, driveConstraints)
                                .build();
                        drive.followTrajectory(trajectory);
                    }
                    else if(gamepad1.dpad_down) {
                        while (gamepad1.dpad_down) sleep(100); // wait for release to debounce
                        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(bottomOfLine, driveConstraints)
                                .build();
                        drive.followTrajectory(trajectory);
                    }
                    else if(gamepad1.x) {
                        while (gamepad1.x) sleep(100); // wait for release to debounce
                        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(home, driveConstraints)
                                .build();
                        drive.followTrajectory(trajectory);
                    }
                    break;
                }
            }

            mecanumLocalizer.update();
            threeWheelTrackingLocalizer.update();
            twoWheelLeftFrontTrackingLocalizer.update();
            twoWheelRightFrontTrackingLocalizer.update();

            telemetry.addData("testMode", testMode.toString());

            telemetry.addLine("MecanumLocalizer");
            telemetry.addData("x", String.format("%.3f", mecanumLocalizer.getPoseEstimate().getX()));
            telemetry.addData("y", String.format("%.3f", mecanumLocalizer.getPoseEstimate().getY()));
            telemetry.addData("heading", String.format("%.3f", Math.toDegrees(mecanumLocalizer.getPoseEstimate().getHeading())));

            telemetry.addLine("threeWheelTrackingLocalizer");
            telemetry.addData("x", String.format("%.3f", threeWheelTrackingLocalizer.getPoseEstimate().getX()));
            telemetry.addData("y", String.format("%.3f", threeWheelTrackingLocalizer.getPoseEstimate().getY()));
            telemetry.addData("heading", String.format("%.3f", Math.toDegrees(threeWheelTrackingLocalizer.getPoseEstimate().getHeading())));

            telemetry.addLine("twoWheelLeftFrontTrackingLocalizer");
            telemetry.addData("x", String.format("%.3f", twoWheelLeftFrontTrackingLocalizer.getPoseEstimate().getX()));
            telemetry.addData("y", String.format("%.3f", twoWheelLeftFrontTrackingLocalizer.getPoseEstimate().getY()));
            telemetry.addData("heading", String.format("%.3f", Math.toDegrees(twoWheelLeftFrontTrackingLocalizer.getPoseEstimate().getHeading())));

            telemetry.addLine("twoWheelRightFrontTrackingLocalizer");
            telemetry.addData("x", String.format("%.3f", twoWheelRightFrontTrackingLocalizer.getPoseEstimate().getX()));
            telemetry.addData("y", String.format("%.3f", twoWheelRightFrontTrackingLocalizer.getPoseEstimate().getY()));
            telemetry.addData("heading", String.format("%.3f", Math.toDegrees(twoWheelRightFrontTrackingLocalizer.getPoseEstimate().getHeading())));

            telemetry.addLine("Encoder Ticks");
            telemetry.addData("leftTicks", String.format("%d", leftPod.getEncoder().getCurrentPosition()));
            telemetry.addData("rightTicks", String.format("%d", rightPod.getEncoder().getCurrentPosition()));
            telemetry.addData("frontTicks", String.format("%d", frontPod.getEncoder().getCurrentPosition()));

            telemetry.addLine("Encoder Position (inches)");
            telemetry.addData("left", String.format("%.3f", leftPod.getPosition()));
            telemetry.addData("right", String.format("%.3f", rightPod.getPosition()));
            telemetry.addData("front", String.format("%.3f", frontPod.getPosition()));

            telemetry.update();
        }

    }

}
