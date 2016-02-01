package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by colbychance on 1/7/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class EncodersTest extends LinearOpMode {

    DcMotor motorRightA;
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor tapeExt1;
    DcMotor tapeExt2;

    Servo clawBody;
    Servo trigger1;
    Servo trigger2;
    Servo tapeAngle;
    Servo plow;
    Servo presser;

    ColorSensor colorSensor;
    final static double FRONT_DIAMETER = 1.8;
    final static double BACK_DIAMETER = 1.5;
    final static double FRONT_CIRCUMFERENCE = FRONT_DIAMETER*Math.PI;
    final static double BACK_CIRCUMFERENCE = BACK_DIAMETER*Math.PI;
    final static double PULSE = 1440;
    final static double RATIO = BACK_DIAMETER/FRONT_DIAMETER;

    @Override
    public void runOpMode() throws InterruptedException {

        motorRightA = hardwareMap.dcMotor.get("motor_fr");
        motorRightB = hardwareMap.dcMotor.get("motor_br");
        motorLeftA = hardwareMap.dcMotor.get("motor_fl");
        motorLeftB = hardwareMap.dcMotor.get("motor_bl");
        tapeExt1 = hardwareMap.dcMotor.get("motor_ext1");
        tapeExt2 = hardwareMap.dcMotor.get("motor_ext2");
        motorLeftA.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);

        clawBody = hardwareMap.servo.get("servo_1");
        trigger1 = hardwareMap.servo.get("servo_2");
        trigger2 = hardwareMap.servo.get("servo_3");
        tapeAngle = hardwareMap.servo.get("servo_4");
        plow = hardwareMap.servo.get("servo_5");
        presser = hardwareMap.servo.get("servo_6");

        colorSensor = hardwareMap.colorSensor.get("sensor_color");



        trigger1.setPosition(.5);
        trigger2.setPosition(.5);
        clawBody.setPosition(1);


        // Wait for the start button to be pressed
        waitForStart();
        drive(0, 1, .5);
        drive(.5, 1, .5);
        turn(-49, 1, .5);
        drive(6.7, 1, .5);
        turn(-54, 1, .5);
        //drive(.75, 1, .5);
        colorPress();

        claw(.5);
        redRamp();

    }

    public void drive(double dist, double pow, double pause) throws InterruptedException {
        double fcount = dist*(PULSE/FRONT_CIRCUMFERENCE);
        DcMotor frontMotors[] = {motorLeftA, motorRightA};
        for(DcMotor x: frontMotors) {
            x.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            x.setTargetPosition((int) fcount);
            x.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }

        motorRightA.setPower(RATIO * pow);
        motorLeftA.setPower(RATIO*pow);
        motorRightB.setPower(pow);
        motorLeftB.setPower(pow);

        while(motorLeftA.isBusy()){
            motorLeftB.setPower(pow);
            motorRightB.setPower(pow);
            telemetry.addData("Distance: ", (motorLeftA.getCurrentPosition()/1440)*FRONT_CIRCUMFERENCE);
        }
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long) (pause * 1000));
    }

    public void turn(double theta, double pow, double pause) throws InterruptedException {

        double period = 3.2;

        if(theta>0){
            motorLeftA.setPower(pow);
            motorRightA.setPower(-pow);
            motorLeftB.setPower(pow);
            motorRightB.setPower(-pow);
        }

        if(theta<0){
            motorLeftA.setPower(-pow);
            motorRightA.setPower(pow);
            motorLeftB.setPower(-pow);
            motorRightB.setPower(pow);
        }

        sleep((long) (1000 * Math.abs((theta / 360) * period)));

        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long)(pause*1000));

    }
    public void claw(double pause) throws InterruptedException {

        clawBody.setPosition(0);

        sleep(1000);

        clawBody.setPosition(1);

        sleep((long) (pause * 1000));

    }

    public void setAngle(double theta, double pause) throws InterruptedException{
        double angle = (3*theta) / 180;
        angle = Range.clip(angle, 0, 1);
        tapeAngle.setPosition(angle);

        sleep((long) (pause * 1000));

    }

    public void tapeExt(double length, double pow, double pause) throws InterruptedException {

        double maxVel=2;

        tapeExt1.setPower(-pow);
        tapeExt2.setPower(-pow*.25);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt1.setPower(0);
        tapeExt2.setPower(0);
        sleep((long) (pause * 1000));

    }

    public void tapePull(double length, double pow, double pause) throws InterruptedException{
        double maxVel=2;

        tapeExt1.setPower(-pow);
        tapeExt2.setPower(-pow*.25);
        sleep(1500);

        motorLeftA.setPower(pow);
        motorRightA.setPower(pow);
        motorLeftB.setPower(pow);
        motorRightB.setPower(pow);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt1.setPower(0);
        tapeExt2.setPower(0);
        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);
        sleep((long) (pause * 1000));
    }

    public void end() throws InterruptedException {

        motorLeftA.setPowerFloat();
        motorLeftB.setPowerFloat();
        motorRightA.setPowerFloat();
        motorRightB.setPowerFloat();

    }

    public void redRamp() throws InterruptedException {
        drive(1, 1, .5);
        turn(50, -1, .5);
        drive(2.4, 1, .5);
        turn(-92, -1, .5);
        drive(3.25, -1, .5);

        setAngle(60.0, .5);
        tapeExt(3, 1, 1.5);
        setAngle(0.0, 2);
        tapePull(4.5, -.5, .5);
    }

    public void colorPress() throws InterruptedException {

        if (colorSensor.red() > colorSensor.blue()) {
            presser.setPosition(.25);
        }
        else if(colorSensor.blue() > colorSensor.blue()) {
            presser.setPosition(.75);
        }
    }

}
