//package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.hardware.RevIMU;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FTCLibMotorController;
//import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.MotorBulkRead;
//import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.SimpleMotorController;
//
//import java.util.function.DoubleSupplier;
//
//@TeleOp(name = "Dev: ShootAndMove", group = "Development")
//public class ShootAndMove extends OpMode {
//
//    public static double    VELOTHRESHOLD = .975,
//            SHOOTERSERVOBACK = .75,
//            SHOOTERSERVOFORWARD = 0;
//    public static int       FLICKDELAY = 400,
//            STAYTIME = FLICKDELAY/2,
//            MAXVELO = 1 /* was 2400. Changed for ease of testing*/,
//            POWER_ADJUSTMENT_DELAY = 250; //the amount of time it has to wait before adjusting power of the shooter motor again again.
//    public static boolean   USESHOOTERENCODER = true;
//    public static int       SHOOTERVERSION = 3;
//
//    FtcDashboard dashboard;
//    public static PIDFCoefficients pidf = new PIDFCoefficients();
//    long lastPressed = 0, lastFlick = 0;
//    double pow = 0;
//    double maxVelo = 0;
//    int motorCount = 0,
//            shotCount = 0;
//    Servo ShooterServo;
//    DcMotorEx ShooterMotorFront, ShooterMotorBack;// not using a FTCLib motor because I want to see how the built in FTC_APP PID controller handles shooting. Maybe I'll transition it over and steal the PID values from the FTC_APP.
////    Log log;
//final double TRACKWIDTH = 7 + (7./16), CENTER_WHEEL_OFFSET = -6.25;
//
//    public double[] motorCPRs  = {
//            383.6,
//            383.6,
//            383.6,
//            383.6
//    };
//
//    FTCLibMotorController dmc;
//    SimpleMotorController imc;
//    final double DISTANCE_PER_PULSE = /*.00056753688;*/(1.37795 * Math.PI) / 8192; //<- old calculation. Decided to use experimentally determined one.
//    final double FORWARD_MULTIPLIER = 1, RIGHT_MULTIPLIER = 1;
//    RevIMU imu;
//    double headingConst;
//    ConstantVeloMecanumOdometry odo;
//    DoubleSupplier heading, leftEncoder, rightEncoder, horizontalEncoder, djx, djy, dr;
//    JSTEncoder le, re, he;
//    Pose2d robotPose, oldPose;
//    public final int IMU_OFFSET = 180;
////    private FtcDashboard dashboard;
//
//
//
//    public void init() {
//        initMotors();
//        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");
//        ShooterServo.setDirection(Servo.Direction.REVERSE);
//
////        FtcDashboard.start();
////        dashboard = FtcDashboard.getInstance();
////        dashboard.setTelemetryTransmissionInterval(25);
////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////        log = new Log("Experiemnt #3", false);
//
//        telemetry.addData(">", "Starting Init(), please wait");
////        FtcDashboard.start();
////        dashboard = FtcDashboard.getInstance();
////        dashboard.setTelemetryTransmissionInterval(25);
////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        imu = new RevIMU(hardwareMap);
//        dmc = new FTCLibMotorController(FTCLibOdo.driveMotorNames.class, hardwareMap, motorCPRs);
//        dmc.reverse(new String[]{"FrontLeftMotor", "BackLeftMotor"});
//        dmc.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        imu.init();
////    imu.invertGyro();
//        imu.reset();
//        le = new JSTEncoder(hardwareMap, FTCLibOdo.driveMotorNames.FrontLeftMotor.name());
//        le.setDistancePerPulse(DISTANCE_PER_PULSE);
//        re = new JSTEncoder(hardwareMap, FTCLibOdo.driveMotorNames.BackRightMotor.name());
//        re.setDistancePerPulse(DISTANCE_PER_PULSE);
//        he = new JSTEncoder(hardwareMap, FTCLibOdo.driveMotorNames.FrontRightMotor.name());
//        he.setDistancePerPulse(DISTANCE_PER_PULSE);
////    he.setInverted(!he.getInverted());
//        MotorBulkRead.MotorBulkMode(LynxModule.BulkCachingMode.MANUAL, hardwareMap);
////    imc = new SimpleMotorController(new String[]{"LeftIntakeMotor", "RightIntakeMotor"}, hardwareMap);
//        headingConst = Math.toRadians(imu.getHeading() *-1);
//        heading = () -> headingConst ; // doing this so that it onyl gets updated once per cycle. I am unsure whther or not I2C devices are affected by bulk reads.
//
//
//
//
//
//        leftEncoder = () -> le.getDistance() * FORWARD_MULTIPLIER;
//        rightEncoder = () -> re.getDistance() * FORWARD_MULTIPLIER;
//        horizontalEncoder = () -> he.getDistance() * FORWARD_MULTIPLIER;
//        odo = new ConstantVeloMecanumOdometry(heading, leftEncoder, rightEncoder, horizontalEncoder, TRACKWIDTH, CENTER_WHEEL_OFFSET);
//
//        djx = () -> gamepad1.left_stick_x;
//        djy = () -> gamepad1.left_stick_y;
//        dr  = () -> gamepad1.right_stick_x;
////                gamepad1.left_trigger > gamepad1.right_trigger ? -gamepad1.left_trigger : gamepad1.right_trigger;
//
//        telemetry.addData(">", "Robot Initialized, have some fun!");
//
//
//
//    }
//
//    public void loop() {
//        MotorBulkRead.clearCache(); // since we have the bulk caching mode on manual, we have to clear the cache every iteration.
//        odo.updatePose();
//        headingConst = Math.toRadians(imu.getHeading());
//
//        oldPose = odo.getPose();
//        robotPose = new Pose2d(oldPose.getTranslation().rotateBy(new Rotation2d(Math.PI/2)), oldPose.getRotation());
//        telemetry.addData("Current Pose", robotPose.toString());
//
//        double[] mpows = driveFieldOriented(djx.getAsDouble(), -djy.getAsDouble(), dr.getAsDouble(), heading.getAsDouble());
//        dmc.setMotor(FTCLibOdo.driveMotorNames.FrontLeftMotor, mpows[0]);
//        dmc.setMotor(FTCLibOdo.driveMotorNames.FrontRightMotor, mpows[1]);
//        dmc.setMotor(FTCLibOdo.driveMotorNames.BackLeftMotor, mpows[2]);
//        dmc.setMotor(FTCLibOdo.driveMotorNames.BackRightMotor, mpows[3]);
//
//        telemetry.addData("Right,Horizon,", re.getCounts() + ", " + he.getCounts());
////        codeImStealingFromRoadRunner();
//
//        if((this.gamepad1.dpad_up || this.gamepad1.dpad_down) && System.currentTimeMillis() - lastPressed > POWER_ADJUSTMENT_DELAY) {
//            pow += this.gamepad1.dpad_up ? .0125 : -.0125;
//            lastPressed = System.currentTimeMillis();
//        }
//
//        ShooterMotorFront.setPower(pow);
//        if(motorCount == 2) ShooterMotorBack.setPower(pow);
//        telemetry.addData("CurrentPower:", pow);
//
//        //little bit of debug because I'm interested. default for 5202 is 10,3,0,0
//        telemetry.addData("Default 5202 PIDF CoEffs:", ShooterMotorFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
//        maxVelo = Math.max(maxVelo, Math.abs(ShooterMotorFront.getVelocity()));
//        telemetry.addData("MaxVelocity:", maxVelo);
//        if(motorCount == 1) telemetry.addData("CurrentVelo:", ShooterMotorFront.getVelocity());
//        else telemetry.addData("CurrentVelo Frontmotor, Backmotor:", ShooterMotorFront.getVelocity() + ", " + ShooterMotorBack.getVelocity());
//        telemetry.addData("ServoPosition", ShooterServo.getPosition());
//        telemetry.addData("Velo in range", Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow - VELOTHRESHOLD && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow + VELOTHRESHOLD);
//        telemetry.addData("Shot Count:", (shotCount % 3 + 1));
//
//
//        if(System.currentTimeMillis() - lastFlick < STAYTIME) ShooterServo.setPosition(1);
//        else if(gamepad1.a && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow * VELOTHRESHOLD && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow * (2- VELOTHRESHOLD) && System.currentTimeMillis() - lastFlick > FLICKDELAY) {
//            lastFlick = System.currentTimeMillis();
//            ShooterServo.setPosition(SHOOTERSERVOFORWARD);
////            shotCount++;
////            log.update();
//
////            log.addData((shotCount % 3 + 1) + ", " + ShooterMotorFront.getVelocity());
//
//        }
//        else ShooterServo.setPosition(SHOOTERSERVOBACK);
//
//        if(gamepad1.b) {
//            ShooterMotorFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        }
//
//        if(gamepad1.x) {
//            pow = -.85;
//        }
//        if(gamepad1.y) {
//            pow = 0;
//        }
//        telemetry.addData("a", gamepad1.a);
//
//        telemetry.addData("TimeWaited", System.currentTimeMillis() - lastFlick > FLICKDELAY);
//        telemetry.addData("HoldingServo", System.currentTimeMillis() - lastFlick < STAYTIME);
////        packet.put("CPS", Math.abs(this.ShooterMotorFront.getVelocity()));
////        dashboard.sendTelemetryPacket(packet);
//    }
//
//
//
//    public enum MotorNames  {
//        ShooterMotorFront,
//        ShooterMotorBack
//    }
//
//
//
////    public double[] motorCPRs = {
////            537.6
////    }; // not currently used but will be if I use FTCLib for the motor controller. Also wrong right now.
//
//    public void initMotors() {
//        switch(SHOOTERVERSION) {
//
//            case 2:
//                pidf = new PIDFCoefficients(1.377, .1377, 0, 11.377);
//                motorCount = 2;
//                break;
//
//            case 3:
//                pidf = new PIDFCoefficients(10, 3, 0, 0); //orig 1.41,.14, 0,14.1
//                motorCount = 1;
//                break;
//
//            case 1:
//            default:
//                pidf = new PIDFCoefficients(10,3,0,0);
//                motorCount = 1;
//                break;
//
//        }
//        ShooterMotorFront = hardwareMap.get(DcMotorEx.class, ShooterTest.MotorNames.ShooterMotorFront.name());
//        ShooterMotorFront.setMode(USESHOOTERENCODER ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ShooterMotorFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        if(motorCount == 2) {
//            ShooterMotorBack = hardwareMap.get(DcMotorEx.class, ShooterTest.MotorNames.ShooterMotorBack.name()); //using single motor design right now
//            ShooterMotorBack.setMode(USESHOOTERENCODER ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            ShooterMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
//            ShooterMotorBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        }
//    }
//    public double[] driveFieldOriented(double jx, double jy, double rot, double gyrangleRad) {
//        double  temp   = jy * Math.cos(gyrangleRad) - jx * Math.sin(gyrangleRad);
//        jx  = jy * Math.sin(gyrangleRad) + jx * Math.cos(gyrangleRad);
//        jy = temp;
//
//        double frontLeft  = jy + jx + rot;
//        double frontRight = jy - jx - rot;
//        double backLeft   = jy - jx + rot;
//        double backRight  = jy + jx - rot;
//
//        double max = Math.abs(frontLeft);
//        if(Math.abs(frontRight) > max) max = Math.abs(frontRight);
//        if(Math.abs(backLeft) > max) max = Math.abs(backLeft);
//        if(Math.abs(backRight) > max) max = Math.abs(backRight);
//
//        if(max > 1) {
//            frontRight /= max;
//            frontLeft  /= max;
//            backRight  /= max;
//            backLeft   /= max;
//        }
//        return new double[]{frontLeft, frontRight, backLeft, backRight};
//    }
//
//    public enum driveMotorNames {
//        FrontLeftMotor,
//        FrontRightMotor,
//        BackLeftMotor,
//        BackRightMotor
//    }
//}
//
//
