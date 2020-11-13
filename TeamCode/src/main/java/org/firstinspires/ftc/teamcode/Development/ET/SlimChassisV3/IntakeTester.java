package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class IntakeTester extends OpMode {
    DcMotor topMotor, bottomMotor;
    double pow = 0;
    long lastPress = 0;
    @Override
    public void init() {
        topMotor = hardwareMap.dcMotor.get("topMotor");
        bottomMotor = hardwareMap.dcMotor.get("bottomMotor");

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        topMotor.setPower(-gamepad1.left_stick_y);
        bottomMotor.setPower(-gamepad1.left_stick_y);
    }
}
