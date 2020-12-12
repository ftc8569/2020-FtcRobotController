package org.firstinspires.ftc.teamcode.Development.CR.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private OdometryPod parallelPod, perpendicularPod;
    private IHeadingProvider headingProvider;

    public TwoWheelTrackingLocalizer(OdometryPod parallelPod, OdometryPod perpendicularPod, IHeadingProvider headingProvider) {
        super(Arrays.asList(parallelPod.getPose(), perpendicularPod.getPose()));
        this.parallelPod = parallelPod;
        this.perpendicularPod = perpendicularPod;
        this.headingProvider = headingProvider;
    }

    @Override
    public double getHeading() {
        return headingProvider.getRawExternalHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(parallelPod.getPosition(), perpendicularPod.getPosition());
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(parallelPod.getVelocity(), perpendicularPod.getVelocity());
    }
}
