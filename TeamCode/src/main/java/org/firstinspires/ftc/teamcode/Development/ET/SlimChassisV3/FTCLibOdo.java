package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.JSTEncoder;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.ConstantVeloMecanumOdometry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FTCLibMotorController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.SimpleMotorController;
import org.firstinspires.ftc.teamcode.Development.ET.util.DashboardUtil;

import java.util.function.DoubleSupplier;

@TeleOp(group = "ET")
public class FTCLibOdo extends OpMode {

    final double TRACKWIDTH = 7 + (7./16), CENTER_WHEEL_OFFSET = -6.25;

    public double[] motorCPRs  = {
            383.6,
            383.6,
            383.6,
            383.6
    };

    FTCLibMotorController dmc;
    SimpleMotorController imc;
    final double DISTANCE_PER_PULSE = /*.00056753688;*/(1.37795 * Math.PI) / 8192; //<- old calculation. Decided to use experimentally determined one.
    final double FORWARD_MULTIPLIER = 1, RIGHT_MULTIPLIER = 1;
    RevIMU imu;
    double headingConst;
    ConstantVeloMecanumOdometry odo;
    DoubleSupplier heading, leftEncoder, rightEncoder, horizontalEncoder, djx, djy, dr;
    JSTEncoder le, re, he;
    Pose2d robotPose, oldPose;
    private FtcDashboard dashboard;

    public void start() {
        telemetry.addData(">", "Starting Init(), please wait");
    }

    public void init() {
        FtcDashboard.start();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    imu = new RevIMU(hardwareMap);
    dmc = new FTCLibMotorController(driveMotorNames.class, hardwareMap, motorCPRs);
    dmc.reverse(new String[]{"FrontLeftMotor", "BackLeftMotor"});
    dmc.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    imu.init();
//    imu.invertGyro();
    imu.reset();
    le = new JSTEncoder(hardwareMap, driveMotorNames.FrontLeftMotor.name());
    le.setDistancePerPulse(DISTANCE_PER_PULSE);
    re = new JSTEncoder(hardwareMap, driveMotorNames.BackRightMotor.name());
    re.setDistancePerPulse(DISTANCE_PER_PULSE);
    he = new JSTEncoder(hardwareMap, driveMotorNames.FrontRightMotor.name());
    he.setDistancePerPulse(DISTANCE_PER_PULSE);
//    he.setInverted(!he.getInverted());
    MotorBulkRead.MotorBulkMode(LynxModule.BulkCachingMode.MANUAL, hardwareMap);
//    imc = new SimpleMotorController(new String[]{"LeftIntakeMotor", "RightIntakeMotor"}, hardwareMap);
    headingConst = Math.toRadians(imu.getHeading());
    heading = () -> headingConst; // doing this so that it onyl gets updated once per cycle. I am unsure whther or not I2C devices are affected by bulk reads.





    leftEncoder = () -> le.getDistance() * FORWARD_MULTIPLIER;
    rightEncoder = () -> re.getDistance() * FORWARD_MULTIPLIER;
    horizontalEncoder = () -> he.getDistance() * FORWARD_MULTIPLIER;
    odo = new ConstantVeloMecanumOdometry(heading, leftEncoder, rightEncoder, horizontalEncoder, TRACKWIDTH, CENTER_WHEEL_OFFSET);

    djx = () -> gamepad1.left_stick_x;
    djy = () -> gamepad1.left_stick_y;
    dr  = () -> gamepad1.left_trigger > gamepad1.right_trigger ? -gamepad1.left_trigger : gamepad1.right_trigger;

    telemetry.addData(">", "Robot Initialized, have some fun!");



    }

    public void loop() {
    MotorBulkRead.clearCache(); // since we have the bulk caching mode on manual, we have to clear the cache every iteration.
    odo.updatePose();
    headingConst = Math.toRadians(imu.getHeading());

    oldPose = odo.getPose();
    robotPose = new Pose2d(oldPose.getTranslation().rotateBy(new Rotation2d(Math.PI/2)), oldPose.getRotation());
    telemetry.addData("Current Pose", robotPose.toString());

    double[] mpows = driveFieldOriented(djx.getAsDouble(), -djy.getAsDouble(), dr.getAsDouble(), heading.getAsDouble());
    dmc.setMotor(driveMotorNames.FrontLeftMotor, mpows[0]);
    dmc.setMotor(driveMotorNames.FrontRightMotor, mpows[1]);
    dmc.setMotor(driveMotorNames.BackLeftMotor, mpows[2]);
    dmc.setMotor(driveMotorNames.BackRightMotor, mpows[3]);

    telemetry.addData("Right,Horizon,", re.getCounts() + ", " + he.getCounts());
    codeImStealingFromRoadRunner();

    }

    //outputs correct motor values in order FL, FR, BL, BR
    public double[] driveFieldOriented(double jx, double jy, double rot, double gyrangleRad) {
        double  temp   = jy * Math.cos(gyrangleRad) - jx * Math.sin(gyrangleRad);
        jx  = jy * Math.sin(gyrangleRad) + jx * Math.cos(gyrangleRad);
        jy = temp;

        double frontLeft  = jy + jx + rot;
        double frontRight = jy - jx - rot;
        double backLeft   = jy - jx + rot;
        double backRight  = jy + jx - rot;

        double max = Math.abs(frontLeft);
        if(Math.abs(frontRight) > max) max = Math.abs(frontRight);
        if(Math.abs(backLeft) > max) max = Math.abs(backLeft);
        if(Math.abs(backRight) > max) max = Math.abs(backRight);

        if(max > 1) {
            frontRight /= max;
            frontLeft  /= max;
            backRight  /= max;
            backLeft   /= max;
        }
        return new double[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void stop() {

    }

    public void codeImStealingFromRoadRunner() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();



        packet.put("x",robotPose.getTranslation().getX());
        packet.put("y", robotPose.getTranslation().getY());
        packet.put("heading", robotPose.getHeading());


        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay,  new com.acmerobotics.roadrunner.geometry.Pose2d(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), robotPose.getHeading()));
        dashboard.sendTelemetryPacket(packet);
    }

    public enum driveMotorNames {
        FrontLeftMotor,
        FrontRightMotor,
        BackLeftMotor,
        BackRightMotor
    }

}
