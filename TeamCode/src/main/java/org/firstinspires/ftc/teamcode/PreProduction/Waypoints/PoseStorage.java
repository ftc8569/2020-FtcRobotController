package org.firstinspires.ftc.teamcode.PreProduction.Waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//It's not really necessary to store this in it's own class but I'm pretty sure this is the safer
// way and it's also how roadrunner recommends so I'll do it like this.
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}
