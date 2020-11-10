package org.firstinspires.ftc.teamcode.PreProduction;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.ETControl.ShooterController;
import org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3.SlimChassisV3Controller;

import java.util.function.DoubleSupplier;

@Disabled
@TeleOp(name = "Pre: ScrimmageTeleOp", group = "Pre-Production")
public class ScrimmageTeleOp extends OpMode {
    FieldOrientedDrive fod;
    ShooterController sc;
    DoubleSupplier djx = () -> gamepad1.left_stick_x, djy = () -> -gamepad1.left_stick_y, dr = () -> gamepad1.right_stick_x, heading;
    RevIMU imu;

//    public final double

    public void init() {
        imu.init();
        heading = () -> imu.getHeading();
        fod = new FieldOrientedDrive(SlimChassisV3Controller.class, hardwareMap, heading, djx, djy, dr);
    }

    public void loop() {

    }
}
