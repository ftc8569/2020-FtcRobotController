package org.firstinspires.ftc.teamcode.ET.SlimChassisV3;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "GoBildaExample", group = "ET")
public class GoBildaExample extends Init {

    static final double TIMETOFULLPOWER = 0.4;
    static double oldLeftStickX, oldLeftStickY, forward, right;
    static double positionStartTime = System.currentTimeMillis() / 1000.0;
    static double currentPower = 0.2;

    public void init() {
        super.init();


    }


    public void loop() {
//        curveIt( -gamepad1.left_stick_y);
//        double clockwise = gamepad1.right_trigger >= gamepad1.left_trigger ? gamepad1.right_trigger : -gamepad1.left_trigger;
//        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(clockwise) > .1) {
//            motorPower(motornames.FrontLeftMotor, forward + clockwise);
//            motorPower(motornames.BackLeftMotor, forward + clockwise);
//            motorPower(motornames.FrontRightMotor, forward - clockwise);
//            motorPower(motornames.BackRightMotor, forward - clockwise);
//        }
//         else stopAllMotors();



            motorPower(motornames.FrontLeftMotor, currentPower);
            motorPower(motornames.BackLeftMotor, currentPower);
            motorPower(motornames.FrontRightMotor, currentPower);
            motorPower(motornames.BackRightMotor, currentPower);
            for(int i = 0; i < motorNames.size(); i++ ) {
                telemetry.addData("Power of " + motorNames.get(i) + " is: ", motors.get(i).getPower());
            }
    }

    public void stop() {
        stopAllMotors();
    }


    //REMINDER TO SELF: when adding in X movement, re-add all X curving and mentions of the X axis
    public void curveIt(double leftStickY) {
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (Math.abs(leftStickY - oldLeftStickY) > .1) {
            positionStartTime = currentTime; }
        double elapsed = currentTime - positionStartTime;
        double portion = elapsed / TIMETOFULLPOWER;
        double scaleFactor = Math.pow(Math.min(portion, 1), 1);
//        right = leftStickX * scaleFactor;
         forward = Math.abs(leftStickY) > .1 ? leftStickY : 0 * scaleFactor;
//        System.out.printf("elapsed=%3.1f scaleFactor=%3.1f X=%3.2f Y=%3.1f \n", elapsed, scaleFactor, forward);
//        oldLeftStickX = leftStickX;
        oldLeftStickY = leftStickY;
        telemetry.addData("RawLeftStickY", leftStickY);
        telemetry.addData("LeftStickYAdj", forward);
    }
}