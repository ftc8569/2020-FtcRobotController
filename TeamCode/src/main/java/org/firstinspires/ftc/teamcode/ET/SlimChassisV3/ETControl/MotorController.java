package org.firstinspires.ftc.teamcode.ET.SlimChassisV3.ETControl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public interface MotorController {

    int findMotor(String motorname);
    int findMotor(Enum motorname);

    void setMotor(String motorname, double pow);
    void setMotor(int mIndex, double pow);
    void setMotor(String[] motornames, double pow);
    void setMotor(Enum motorname, double pow);

    void zeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb);
    void zeroPowerBehavior(String motorname, DcMotor.ZeroPowerBehavior zpb);
    void zeroPowerBehavior(Enum motorname, DcMotor.ZeroPowerBehavior zpb);
    void zeroPowerBehavior(int motorindex, DcMotor.ZeroPowerBehavior zpb);
    void zeroPowerBehavior(String[] motornames, DcMotor.ZeroPowerBehavior zpb);

    void setDirection(DcMotorSimple.Direction dir);
    void setDirection(String motorname, DcMotorSimple.Direction dir);
    void setDirection(Enum motorname, DcMotorSimple.Direction dir);
    void setDirection(int motorindex, DcMotorSimple.Direction dir);
    void setDirection(String[] motornames, DcMotorSimple.Direction dir);

    void reverse();
    void reverse(String motorname);
    void reverse(Enum motorname);
    void reverse(int motorindex);
    void reverse(String[] motornames);


}
