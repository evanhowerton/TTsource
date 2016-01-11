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

public class CS3  extends LinearOpMode{
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("cs");
        colorSensor.enableLed(true);
        waitOneFullHardwareCycle();
        waitForStart();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        while(opModeIsActive()) {
            telemetry.addData("Text", "*** Robot Data***");
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            //telemetry.addData("Reflectance", "Reflectance Value:  " + reflectance);
            telemetry.addData("Red", "Red Val: " + colorSensor.red());
            telemetry.addData("Blue", "Blue Val: " + colorSensor.blue());
            telemetry.addData("Green", "Green Val: " + colorSensor.green());

            waitOneFullHardwareCycle();
        }
    }
}
