package org.firstinspires.ftc.teamcode.Development.CR.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import kotlin.NotImplementedError;

@Config
public class VuforiaMecanumLocalizer implements Localizer {

    public enum LocalizationMode {
        VUFORIA_ONLY,
        MECANUM_ODOMETRY_ONLY,
        PREFER_VUFORIA,
        CONFIRMONSTOP_WITH_VUFORIA
    }

    // Used by FTCDashboard @Config
    public static LocalizationMode LOCALIZATIONMODE = LocalizationMode.MECANUM_ODOMETRY_ONLY;
    public static double STOPPED_VELOCITY = 0.05;

    private MecanumDrive.MecanumLocalizer mecanumLocalizer;
    private VuforiaDriveLocalizer vuforiaDriveLocalizer;
    private Pose2d currentPoseEstimate = new Pose2d(0,0,0);
    private Pose2d currentVelocityEstimate = new Pose2d(0,0,0);
    private boolean isInStoppedMode = true;
    private double stoppedVuforiaSamples = 0.0;
    private double stoppedVuforiaXTotal = 0.0;
    private double stoppedVuforiaYTotal = 0.0;
    private double stoppedVuforiaHeadingTotal = 0.0;

    public VuforiaMecanumLocalizer(HardwareMap hardwareMap, MecanumDrive.MecanumLocalizer mecanumLocalizer){
        this(hardwareMap, mecanumLocalizer, false);
    }
    public VuforiaMecanumLocalizer(HardwareMap hardwareMap, MecanumDrive.MecanumLocalizer mecanumLocalizer, boolean showCameraFeed){
        this.mecanumLocalizer = mecanumLocalizer;
        this.vuforiaDriveLocalizer = new VuforiaDriveLocalizer(hardwareMap, showCameraFeed);
        this.vuforiaDriveLocalizer.activate();
    }

    public LocalizationMode getLocalizationMode() {
        return LOCALIZATIONMODE;
    }


    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return currentPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        currentPoseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return currentVelocityEstimate;
    }

    @Override
    public void update() {

        mecanumLocalizer.update();
        vuforiaDriveLocalizer.update();

        switch (this.getLocalizationMode()){
            case MECANUM_ODOMETRY_ONLY:
                currentPoseEstimate = mecanumLocalizer.getPoseEstimate();
                currentVelocityEstimate = mecanumLocalizer.getPoseVelocity();
                break;
            case VUFORIA_ONLY:
                currentPoseEstimate = vuforiaDriveLocalizer.getPoseEstimate();
                currentVelocityEstimate = vuforiaDriveLocalizer.getPoseVelocity();
            case PREFER_VUFORIA:
                if(vuforiaDriveLocalizer.isTargetVisible()){
                    currentPoseEstimate = vuforiaDriveLocalizer.getPoseEstimate();
                    currentVelocityEstimate = vuforiaDriveLocalizer.getPoseVelocity();
                    mecanumLocalizer.setPoseEstimate(currentPoseEstimate);
                }
                else {
                    currentPoseEstimate = mecanumLocalizer.getPoseEstimate();
                    currentVelocityEstimate = mecanumLocalizer.getPoseVelocity();
                }
                break;
            case CONFIRMONSTOP_WITH_VUFORIA:
                if(isInStoppedMode) {
                    if(!isRobotStopped()){
                        isInStoppedMode = false;
                    }
                    else if(vuforiaDriveLocalizer.isTargetVisible()){
                        Pose2d vuPose = vuforiaDriveLocalizer.getPoseEstimate();
                        stoppedVuforiaSamples += 1.0;
                        stoppedVuforiaXTotal += vuPose.getX();
                        stoppedVuforiaYTotal += vuPose.getY();
                        stoppedVuforiaHeadingTotal += vuPose.getHeading();

                        double averageStoppedX = stoppedVuforiaXTotal / stoppedVuforiaSamples;
                        double averageStoppedY = stoppedVuforiaYTotal / stoppedVuforiaSamples;
                        double averageStoppedHeading = stoppedVuforiaHeadingTotal / stoppedVuforiaSamples;

                        mecanumLocalizer.setPoseEstimate(new Pose2d(averageStoppedX,averageStoppedY,averageStoppedHeading));
                    }
                }
                else {
                    if(isRobotStopped()) {
                        isInStoppedMode = true;
                        stoppedVuforiaSamples = 0.0;
                        stoppedVuforiaXTotal = 0.0;
                        stoppedVuforiaYTotal = 0.0;
                        stoppedVuforiaHeadingTotal = 0.0;
                    }
                }
                currentPoseEstimate = mecanumLocalizer.getPoseEstimate();
                currentVelocityEstimate = mecanumLocalizer.getPoseVelocity();
            default:
                throw new NotImplementedError();
        }
    }
    private boolean isRobotStopped() {
        return  (mecanumLocalizer.getPoseVelocity().getX() <= STOPPED_VELOCITY && mecanumLocalizer.getPoseVelocity().getY() <= STOPPED_VELOCITY);
    }
}
