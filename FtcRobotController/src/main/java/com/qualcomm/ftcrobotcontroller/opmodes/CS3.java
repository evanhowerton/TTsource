package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by colbychance on 1/11/16.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import javafx.scene.shape.Line;

public class CS3  extends LinearOpMode{
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("cs");
        colorSensor.enableLed(true);
        waitOneFullHardwareCycle();
        waitForStart();

        double redv = colorSensor.red();
        double bluev = colorSensor.blue();
        double greenv = colorSensor.green();


        while(opModeIsActive()) {
            telemetry.addData("Text", "*** Robot Data***");
            //telemetry.addData("Reflectance", "Reflectance Value:  " + reflectance);
            telemetry.addData("Red", "Red Val: " + redv);
            telemetry.addData("Blue", "Blue Val: " + bluev);
            telemetry.addData("Green", "Green Val: " + greenv);
        }
    }
}
