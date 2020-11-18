/*package org.firstinspires.ftc.teamcode.Development.CR.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@Config
public class IntelT265Localizer implements Localizer {

    // everything here is in meter and radians as that is what the T265 uses.
    public static com.arcrobotics.ftclib.geometry.Pose2d ROBOTSTARTINGPOSE = new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0));
    public static Transform2d ROBOTOFFSET = new Transform2d(new Translation2d(216,0), new Rotation2d(Math.PI));
    public static double ODOMETRYCOVARIANCE = 0.8;

    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    private Transform2d transformToCameraStartingPose = null;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private MecanumDrive.MecanumLocalizer mecanumLocalizer;


    // NOTE the com.arcrobotics.ftclib.geometry.Pose2d and com.acmerobotics.roadrunner.geometry.Pose2d are different
    private Pose2d currentPoseEstimate = new Pose2d(ROBOTSTARTINGPOSE.getX(),ROBOTSTARTINGPOSE.getY(),ROBOTSTARTINGPOSE.getHeading());
    private Pose2d currentVelocityEstimate = new Pose2d(0,0,0);

    public  IntelT265Localizer(HardwareMap hardwareMap, MecanumDrive.MecanumLocalizer mecanumLocalizer){
        // store a copy of the default mecanum localizer. We can use it to feed odometry into the T265 (hopefully)
        this.mecanumLocalizer = mecanumLocalizer;
        initializeT265Camera(hardwareMap);
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return this.currentPoseEstimate;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return this.currentVelocityEstimate;
    }

    @Override
    public void update() {
        // update the wheel positions and velocities from mecanum wheel odometry
//        mecanumLocalizer.update();
//       slamra.sendOdometry(mecanumLocalizer.getPoseVelocity().getX(), mecanumLocalizer.getPoseVelocity().getY());

        // now get an update from the T265
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up != null) {
            com.arcrobotics.ftclib.geometry.Pose2d pose = up.pose.transformBy(transformToCameraStartingPose).transformBy(ROBOTOFFSET);

            // convert meters to inches and store in variables
            currentPoseEstimate = new Pose2d( metersToInches(pose.getX()), metersToInches(pose.getY()), pose.getHeading());
            currentVelocityEstimate = new Pose2d(metersToInches(up.velocity.vxMetersPerSecond),
                    metersToInches(up.velocity.vyMetersPerSecond), up.velocity.omegaRadiansPerSecond);
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        this.currentPoseEstimate = pose2d;
    }

    private void initializeT265Camera(HardwareMap hardwareMap){
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), ODOMETRYCOVARIANCE, hardwareMap.appContext);
            slamra.start();
        }
        while (transformToCameraStartingPose == null) {
            T265Camera.CameraUpdate initialUpdate = slamra.getLastReceivedCameraUpdate();
            if(initialUpdate != null){
                com.arcrobotics.ftclib.geometry.Pose2d cameraStartingPosition = ROBOTSTARTINGPOSE.transformBy(ROBOTOFFSET.inverse());
                transformToCameraStartingPose = cameraStartingPosition.minus(initialUpdate.pose);
            }
        }
    }

    private static double inchesToMeters(double inches){
        return inches / 39.3700787;
    }
    private static double metersToInches(double meters){
        return meters * 39.3700787;
    }

}
*/