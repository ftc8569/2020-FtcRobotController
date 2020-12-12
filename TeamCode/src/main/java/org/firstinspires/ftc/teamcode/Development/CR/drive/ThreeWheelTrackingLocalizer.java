package org.firstinspires.ftc.teamcode.Development.CR.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.android.dx.cf.direct.CodeObserver;
import org.firstinspires.ftc.teamcode.Development.ET.util.Encoder;

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
public class ThreeWheelTrackingLocalizer extends ThreeTrackingWheelLocalizer {
    private OdometryPod leftPod, rightPod, frontPod;

    public ThreeWheelTrackingLocalizer(OdometryPod leftPod, OdometryPod rightPod, OdometryPod frontPod) {
        super(Arrays.asList(leftPod.getPose(), rightPod.getPose(), frontPod.getPose()));
        this.leftPod = leftPod;
        this.rightPod = rightPod;
        this.frontPod = frontPod;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(leftPod.getPosition(), rightPod.getPosition(), frontPod.getPosition());
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(leftPod.getVelocity(), rightPod.getVelocity(), frontPod.getVelocity());
    }
}
