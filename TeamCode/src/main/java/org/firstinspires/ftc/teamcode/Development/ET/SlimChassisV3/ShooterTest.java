package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/*  Current Shooter Version: 3
    Motor count: 1
    Gear ratio including motor gearbox: 1:1
 */

@Config
@TeleOp(group = "Development", name = "Dev: ShooterTest")
@Disabled
public class ShooterTest extends OpMode {





    /*Easily adjustable constants for quick testing and prototyping, now adjustable using FTCDashboard. They still have the naming convention of final variables even though they have to be static for that to work though.*/
    public static double    VELOTHRESHOLD = 0.125,
                            SHOOTERSERVOBACK = .75,
                            SHOOTERSERVOFORWARD = 0;
    public static int       FLICKDELAY = 300,
                            STAYTIME = FLICKDELAY/2,
                            MAXVELO = 1 /* was 2400. Changed for ease of testing*/,
                            POWER_ADJUSTMENT_DELAY = 250; //the amount of time it has to wait before adjusting power of the shooter motor again again.
    public static boolean   USESHOOTERENCODER = true;
    public static int       SHOOTERVERSION = 3;

    FtcDashboard dashboard;
    public static PIDFCoefficients pidf = new PIDFCoefficients();
    long lastPressed = 0, lastFlick = 0;
    double pow = 0;
    double maxVelo = 0;
    int motorCount = 0,
    shotCount = 0;
    Servo ShooterServo;
    DcMotorEx ShooterMotorFront, ShooterMotorBack;// not using a FTCLib motor because I want to see how the built in FTC_APP PID controller handles shooting. Maybe I'll transition it over and steal the PID values from the FTC_APP.
    Log log;


    public void init() {
        initMotors();
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");
        ShooterServo.setDirection(Servo.Direction.REVERSE);

//        FtcDashboard.start();
//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        log = new Log("Experiemnt #3", false);
    }

    public void loop(){
//        TelemetryPacket packet = new TelemetryPacket();
        if((this.gamepad1.dpad_up || this.gamepad1.dpad_down) && System.currentTimeMillis() - lastPressed > POWER_ADJUSTMENT_DELAY) {
            pow += this.gamepad1.dpad_up ? .0125 : -.0125;
            lastPressed = System.currentTimeMillis();
        }

        ShooterMotorFront.setPower(pow);
        if(motorCount == 2) ShooterMotorBack.setPower(pow);
        telemetry.addData("CurrentPower:", pow);

        //little bit of debug because I'm interested. default for 5202 is 10,3,0,0
        telemetry.addData("Default 5202 PIDF CoEffs:", ShooterMotorFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        maxVelo = Math.max(maxVelo, Math.abs(ShooterMotorFront.getVelocity()));
        telemetry.addData("MaxVelocity:", maxVelo);
        if(motorCount == 1) telemetry.addData("CurrentVelo:", ShooterMotorFront.getVelocity());
        else telemetry.addData("CurrentVelo Frontmotor, Backmotor:", ShooterMotorFront.getVelocity() + ", " + ShooterMotorBack.getVelocity());
        telemetry.addData("ServoPosition", ShooterServo.getPosition());
        telemetry.addData("Velo in range", Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow - VELOTHRESHOLD && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow + VELOTHRESHOLD);
        telemetry.addData("Shot Count:", (shotCount % 3 + 1));


        if(System.currentTimeMillis() - lastFlick < STAYTIME) ShooterServo.setPosition(1);
        else if(gamepad1.a && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow - VELOTHRESHOLD && Math.abs(ShooterMotorFront.getVelocity() / MAXVELO) > pow + VELOTHRESHOLD && System.currentTimeMillis() - lastFlick > FLICKDELAY) {
            lastFlick = System.currentTimeMillis();
            ShooterServo.setPosition(SHOOTERSERVOFORWARD);
//            shotCount++;
            log.update();

            log.addData((shotCount % 3 + 1) + ", " + ShooterMotorFront.getVelocity());

        }
        else ShooterServo.setPosition(SHOOTERSERVOBACK);

        if(gamepad1.b) {
            ShooterMotorFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }

        if(gamepad1.x) {
            pow = -.85;
        }
        if(gamepad1.y) {
            pow = 0;
        }
        telemetry.addData("a", gamepad1.a);

        telemetry.addData("TimeWaited", System.currentTimeMillis() - lastFlick > FLICKDELAY);
        telemetry.addData("HoldingServo", System.currentTimeMillis() - lastFlick < STAYTIME);
//        packet.put("CPS", Math.abs(this.ShooterMotorFront.getVelocity()));
//        dashboard.sendTelemetryPacket(packet);
    }



    public enum MotorNames  {
        ShooterMotorFront,
        ShooterMotorBack
    }



    public double[] motorCPRs = {
            537.6
    }; // not currently used but will be if I use FTCLib for the motor controller. Also wrong right now.

    public void initMotors() {
        switch(SHOOTERVERSION) {

            case 2:
                pidf = new PIDFCoefficients(1.377, .1377, 0, 11.377);
                motorCount = 2;
            break;

            case 3:
                pidf = new PIDFCoefficients(10, 3, 0, 0); //orig 1.41,.14, 0,14.1
                motorCount = 1;
            break;

            case 1:
            default:
                pidf = new PIDFCoefficients(10,3,0,0);
                motorCount = 1;
                break;

        }
        ShooterMotorFront = hardwareMap.get(DcMotorEx.class, MotorNames.ShooterMotorFront.name());
        ShooterMotorFront.setMode(USESHOOTERENCODER ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotorFront.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
        if(motorCount == 2) {
            ShooterMotorBack = hardwareMap.get(DcMotorEx.class, MotorNames.ShooterMotorBack.name()); //using single motor design right now
            ShooterMotorBack.setMode(USESHOOTERENCODER ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ShooterMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
            ShooterMotorBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }
}
