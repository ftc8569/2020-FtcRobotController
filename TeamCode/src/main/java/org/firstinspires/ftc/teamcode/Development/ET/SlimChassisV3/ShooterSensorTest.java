package org.firstinspires.ftc.teamcode.Development.ET.SlimChassisV3;

import android.os.Build;
import android.os.Environment;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@RequiresApi(api = Build.VERSION_CODES.O)
@Autonomous(group = "ET")
public class ShooterSensorTest extends LinearOpMode {

    Rev2mDistanceSensor dist;


    List<Double> norings = new ArrayList<>();
    List<Double> onering = new ArrayList<>();
    List<Double> tworings = new ArrayList<>();
    List<Double> threerings = new ArrayList<>();

    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/FIRST";
    Log log;
    DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd");
    LocalDateTime now = LocalDateTime.now();
    public void runOpMode() {
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
        log = new Log("ShooterSensorTest", false);
        log.update();
        waitForStart();

            telemetry.addData(">", "Ready to test at 0 rings. Press a to begin");
            telemetry.update();
            while (!gamepad1.a) {
                sleep(1);
            }
            telemetry.addData(">", "starting data collection");
            telemetry.update();
            log.addData("0 Disks");
            for (int i = 0; i < 1000; i++) {
                log.addData(dist.getDistance(DistanceUnit.INCH));
                log.update();
                telemetry.addData("Count:", i);
                telemetry.addData("Distance:", dist.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            telemetry.addData(">", "Ready to test at 1 rings. Press a to begin");
            telemetry.update();

        while (!gamepad1.a) {
                sleep(1);
            }
        telemetry.addData(">", "starting data collection");
        telemetry.update();
            for (int i = 0; i < 1000; i++) {
                onering.add(dist.getDistance(DistanceUnit.INCH));
                telemetry.addData("Count:", i);
                telemetry.addData("Distance:", dist.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            telemetry.addData(">", "Ready to test at 2 rings. Press a to begin");
             telemetry.update();

        while (!gamepad1.a) {
                sleep(1);
            }
        telemetry.addData(">", "starting data collection");
        telemetry.update();
            for (int i = 0; i < 1000; i++) {
                tworings.add(dist.getDistance(DistanceUnit.INCH));
                telemetry.addData("Count:", i);
                telemetry.addData("Distance:", dist.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            telemetry.addData(">", "Ready to test at 3 rings. Press a to begin");
            telemetry.update();

        while (!gamepad1.a) {
                sleep(1);
            }
        telemetry.addData(">", "starting data collection");
        telemetry.update();
            for (int i = 0; i < 1000; i++) {
                threerings.add(dist.getDistance(DistanceUnit.INCH));
                telemetry.addData("Count:", i);
                telemetry.addData("Distance:", dist.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            telemetry.addData("Collection done,", "attempting to write file");
            telemetry.update();
            log.addData("NoRings" + "," + "OneRing" + ", " + "TwoRings" + ", "+ "ThreeRings");
        for(int i = 0; i < norings.size(); i++) {
            log.addData(Arrays.asList(norings.get(i), onering.get(i), tworings.get(i), threerings.get(i)));
        }
            telemetry.addData("File Written!", "Yay");
            telemetry.update();
            while(opModeIsActive()) {}
    }}




