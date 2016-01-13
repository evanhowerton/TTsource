package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by colbychance on 1/7/16.
 */

        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;

        import com.qualcomm.ftcrobotcontroller.R;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.LED;
        import com.qualcomm.robotcore.hardware.TouchSensor;
public class COLBYTEST extends LinearOpMode {
    public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};
    public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;

    ColorSensor colorSensor;
    //DeviceInterfaceModule cdim;
    LED led;

    @Override
    public void runOpMode() throws InterruptedException {
        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor = hardwareMap.colorSensor.get("cs");
        led = hardwareMap.led.get("led");

        waitForStart();

        float hsvValues[] = {0,0,0};
        final float values[] = hsvValues;

        while (opModeIsActive()) {
            Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);
            colorSensor.enableLed(true);
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            waitOneFullHardwareCycle();
        }
    }
}
