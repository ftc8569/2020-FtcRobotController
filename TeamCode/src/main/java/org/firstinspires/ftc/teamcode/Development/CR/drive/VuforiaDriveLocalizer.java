package org.firstinspires.ftc.teamcode.Development.CR.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaDriveLocalizer implements Localizer {
    public static double WEBCAM_X_ROTATE = 0;   // in degrees
    public static double WEBCAM_Y_ROTATE = -90; // in degrees
    public static double WEBCAM_Z_ROTATE = 180; // in degrees

    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    public static double WEBCAM_FORWARD_DISPLACEMENT  = 200;   // in mm from center of robot on floor
    public static double WEBCAM_VERTICAL_DISPLACEMENT = 40;    // in mm from center of robot on floor
    public static double WEBCAM_LEFT_DISPLACEMENT     = 155;   // in mm from center of robot on floor

    private static final float mmPerInch        = 25.4f;
    private static final String VUFORIA_KEY = "AVrMK4D/////AAABmRoNuz4dD0dBv9Yw0fEt2597kETRFjcvY3bC9VW1k9vNTnWuBQbY0y3m0BN3P2oB2Z0PnC/a5G+DpqZA8lWQtuT04lWJLft/CdXpig0DlzjkA8pLczBVhd3FhLdvkMfXtusOEw3mS1arDE8rEsOvvyQ/d7Dpw8WleIzu4buqw8eS80Jlm2tCQWEC09iKTUFfSJPnisfBX7XA59V816yrxf5WgqiMXUpbRJYbCP75PUfSo0I3SUJg4iXYDu8k7xRhQEMbNC2wZXsSq3NJYPwj73kNU7A0DkFdrjdlvfVIUd7i1GmsKTRSTz6a2Xqmq9ffGG0N0Q6LS49Oy5ebzXcEr6fJLK/XsxuMyVsxu9vqygDR";
    private static final float mmTargetHeight   = inchesToMM(6);  // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = inchesToMM(72);
    private static final float quadField  = inchesToMM(36);

    // Class Members
    private org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer vuforia = null;
    private WebcamName webcamName = null;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackable blueTowerGoalTarget, redTowerGoalTarget, redAllianceTarget, blueAllianceTarget, frontWallTarget;
    VuforiaTrackables targetsUltimateGoal;
    boolean targetVisible = false;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry telemetry = dashboard.getTelemetry();
    private Pose2d previousPoseEstimate = new Pose2d(0,0,0);
    private Pose2d currentPoseEstimate = new Pose2d(0,0,0);
    private Pose2d currentVelocityEstimate = new Pose2d(0,0,0);
    private String currentVisibleTargetName = "none";
    private final NanoClock clock = NanoClock.system();
    private double previousUpdateTime = clock.seconds();

    public VuforiaDriveLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }
    public VuforiaDriveLocalizer(HardwareMap hardwareMap, boolean showCameraFeed) {
        //initialize vuforia parameters
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer.Parameters parameters;
        if(showCameraFeed) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        else {
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

    }

    public void activate(){
        targetsUltimateGoal.activate();
    }
    public void deactivate(){
        targetsUltimateGoal.deactivate();
    }
    public boolean isTargetVisible() {
        return targetVisible;
    }
    public String getCurrentVisibleTargetName() {
        return currentVisibleTargetName;
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

        // this is a separate function because we will eventually use a pan/tilt (mostly pan) for the webcam
        // and this will update the transforms given the inverse kinematics of the pan/tilt
        updateCameraPose();

        // check all the trackable targets to see which one (if any) is visible.
        boolean targetFound = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetFound = true;
                currentVisibleTargetName = trackable.getName();

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    double updateTime = clock.seconds();

                    // get the current pose from the transform (values from vuforia are in mm and degrees - convert to inches and radians)
                    double robotX = mmToInches(robotLocationTransform.getTranslation().get(0));
                    double robotY = mmToInches(robotLocationTransform.getTranslation().get(1));
                    Orientation rotation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);
                    double robotHeadingInRadians = rotation.thirdAngle * Math.PI * 2.0/360.0;
                    previousPoseEstimate = currentPoseEstimate;
                    currentPoseEstimate = new Pose2d(robotX, robotY, robotHeadingInRadians);

                    double robotVx = (currentPoseEstimate.getX() - previousPoseEstimate.getX())/(updateTime - previousUpdateTime);
                    double robotVy = (currentPoseEstimate.getY() - previousPoseEstimate.getY())/(updateTime - previousUpdateTime);
                    double robotVh = (currentPoseEstimate.getHeading() - previousPoseEstimate.getHeading())/(updateTime - previousUpdateTime);
                    currentVelocityEstimate = new Pose2d(robotVx, robotVy, robotVh);
                    previousUpdateTime = updateTime;
                }
                break;
            }
        }
        targetVisible = targetFound;
        if(!targetVisible)
            currentVisibleTargetName = "none";
    }

    private void updateCameraPose(){
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation((float)WEBCAM_FORWARD_DISPLACEMENT, (float)WEBCAM_LEFT_DISPLACEMENT, (float)WEBCAM_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, (float)WEBCAM_Y_ROTATE, (float)WEBCAM_Z_ROTATE, (float)WEBCAM_X_ROTATE));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, cameraLocationOnRobot);
        }
    }
    private static float mmToInches(float mm) {
        return  mm / mmPerInch;
    }
    private static float inchesToMM(float inches) {
        return  inches * mmPerInch;
    }
}
