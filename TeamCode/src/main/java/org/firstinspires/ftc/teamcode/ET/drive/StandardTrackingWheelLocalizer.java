package org.firstinspires.ftc.teamcode.ET.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = .6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7+(7./16); // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.25; // in; offset of the lateral wheel

    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;

    public static double X_MULTIPLIER = 1, Y_MULTIPLIER = 1;
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.get(DcMotorEx.class, "FrontLeftMotor");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "BackRightMotor");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "FrontRightMotor");
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * Y_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * Y_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * X_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getVelocity()),
                encoderTicksToInches(rightEncoder.getVelocity()),
                encoderTicksToInches(frontEncoder.getVelocity())
        );
    }
}
