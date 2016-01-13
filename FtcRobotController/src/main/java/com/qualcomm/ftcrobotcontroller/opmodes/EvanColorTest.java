package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by colbychance on 1/7/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class EvanColorTest extends LinearOpMode {

    //OpticalDistanceSensor ODS;
    //TouchSensor touchSensor;
    ColorSensor colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        //ODS = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        //touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        double redv = colorSensor.red();
        double bluev = colorSensor.blue();
        double greenv = colorSensor.green();

        for (int i = 0; i < 200; i++) {

            telemetry.addData("Text", "*** Robot Data***");
            //telemetry.addData("Reflectance", "Reflectance Value:  " + reflectance);
            telemetry.addData("Red", "Red Val: " + redv);
            telemetry.addData("Blue", "Blue Val: " + bluev);
            telemetry.addData("Green", "Green Val: " + greenv);

            sleep(100);
        }

    }
}
