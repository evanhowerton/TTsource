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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class CS3  extends LinearOpMode{

    ColorSensor colorSensor;
    OpticalDistanceSensor ODS;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("cs");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor.enableLed(true);
        waitOneFullHardwareCycle();
        waitForStart();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        while(opModeIsActive()) {
            colorSensor.enableLed(true);
            telemetry.addData("Text", "*** Robot Data***");
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            telemetry.addData("Refl", "Reflectance Value:  " + ODS.getLightDetected());
            telemetry.addData("Red", "Red Val: " + colorSensor.red());
            telemetry.addData("Blue", "Blue Val: " + colorSensor.blue());
            telemetry.addData("Green", "Green Val: " + colorSensor.green());

            waitOneFullHardwareCycle();
        }
    }
}
