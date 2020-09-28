package org.firstinspires.ftc.teamcode.ET.SlimChassisV3.StaticVaribaleTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.security.acl.Group;

@Disabled

@Autonomous(group = "ET")
public class StaticVariableAuto extends OpMode {

    public void init() {
        StaticVariableTele.number = 2;

    }

    public void loop() {


    }

    public void stop() {


    }
}
