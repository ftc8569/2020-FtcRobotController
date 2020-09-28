package org.firstinspires.ftc.teamcode.ET.SlimChassisV3;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp
public class SimpleTeleopThatIsntStupidlyOvercomplicated extends OpMode {

    DcMotorEx FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    public BNO055IMU imu;
    double forward, right, clockwise, frontLeft, frontRight, backLeft, backRight, max, degrees;
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    Orientation orientation;

    @Override
    public void init() {
        FrontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotorEx.class, "BackLeftMotor");

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParameters);

        telemetry.addData("Init Status", "Initialized");
    }

    @Override
    public void loop() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        degrees = orientation.firstAngle;

        telemetry.addData("ZYX angle", orientation.firstAngle + ", " + orientation.secondAngle + ", " + orientation.thirdAngle);

        double leftStickY = this.gamepad1.left_stick_y,
                leftStickX = this.gamepad1.left_stick_x,
                leftTrigger = this.gamepad1.left_trigger,
                rightTrigger = this.gamepad1.right_trigger;

        forward = 0;
        right = 0;
        clockwise = 0;

        if(Math.abs(leftStickY) > .05) {
            forward = -leftStickY;
        } else forward = 0;

        if(Math.abs(leftStickX) > .05) {
            right  =  leftStickX;
        } else right = 0;


        if(leftTrigger > .05 || rightTrigger > .05) {
            clockwise = (leftTrigger >= rightTrigger) ?
                    -leftTrigger : rightTrigger;
        }

        if(gamepad1.dpad_up) forward = 1;
        if(gamepad1.dpad_down) forward = -1;
        if(gamepad1.dpad_left) right = -1;
        if(gamepad1.dpad_right) right = 1;
        double radians = Math.toRadians(degrees); /* conv 0-360 to 0-2 pi */

        double  temp   = forward * Math.cos(radians) - right * Math.sin(radians);
        right  = forward * Math.sin(radians) + right * Math.cos(radians);
        forward = temp;

        frontLeft  = forward - right + clockwise;
        frontRight = forward + right - clockwise;
        backLeft   = forward + right + clockwise;
        backRight  = forward - right - clockwise;

        max = Math.abs(frontLeft);
        if(Math.abs(frontRight) > max) max = Math.abs(frontRight);
        if(Math.abs(backLeft) > max) max = Math.abs(backLeft);
        if(Math.abs(backRight) > max) max = Math.abs(backRight);

        if(max > 1) {
            frontRight /= max;
            frontLeft  /= max;
            backRight  /= max;
            backLeft   /= max;
        }

        if(Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1 || gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) {
            FrontLeftMotor.setPower(frontLeft);
            FrontRightMotor.setPower(frontRight);
            BackLeftMotor.setPower(backLeft);
            BackRightMotor.setPower(backRight);
        } else {
            FrontLeftMotor.setPower(0);
            FrontRightMotor.setPower(0);
            BackLeftMotor.setPower(0);
            BackRightMotor.setPower(0);
        }
    }

    public void stop() {
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);
    }
}
