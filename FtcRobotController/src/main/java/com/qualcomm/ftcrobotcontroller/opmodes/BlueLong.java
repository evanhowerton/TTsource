package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class BlueLong extends LinearOpMode {
    DcMotor motorRightA;
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor tapeExt;
    Servo clawBody;
    Servo trigger;
    Servo tapeAngle;

    //hello2


    @Override
    public void runOpMode() throws InterruptedException {

        motorRightA = hardwareMap.dcMotor.get("motor_fr");
        motorRightB = hardwareMap.dcMotor.get("motor_br");
        motorLeftA = hardwareMap.dcMotor.get("motor_fl");
        motorLeftB = hardwareMap.dcMotor.get("motor_bl");
        tapeExt = hardwareMap.dcMotor.get("motor_ext");
        motorLeftA.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);

        clawBody = hardwareMap.servo.get("servo_1");
        trigger = hardwareMap.servo.get("servo_2");
        tapeAngle = hardwareMap.servo.get("servo_3");


        trigger.setPosition(.5);
        clawBody.setPosition(1);


        // Wait for the start button to be pressed
        waitForStart();
        drive(0, 1, .5);
        drive(1, 1, .5);
        turn(-48, 1, .5);
        drive(8, 1, .5);
        turn(-51, 1, .5);

        claw(.5);
        blueRamp();
    }

    public void drive(double dist, double pow, double pause) throws InterruptedException {

        double maxVel= 2.31;

        motorLeftA.setPower(-pow);
        motorRightA.setPower(-pow);
        motorLeftB.setPower(-pow);
        motorRightB.setPower(-pow);

        sleep((long) (1000 * (Math.abs(dist / (maxVel * pow)))));

        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long) (pause * 1000));
    }

    public void turn(double theta, double pow, double pause) throws InterruptedException {

        double period = 3.2;

        if(theta>0){
            motorLeftA.setPower(-pow);
            motorRightA.setPower(pow);
            motorLeftB.setPower(-pow);
            motorRightB.setPower(pow);
        }

        if(theta<0){
            motorLeftA.setPower(pow);
            motorRightA.setPower(-pow);
            motorLeftB.setPower(pow);
            motorRightB.setPower(-pow);
        }

        sleep((long) (1000 * Math.abs((theta / 360) * period)));

        motorLeftA.setPower(0);
        motorRightA.setPower(0);
        motorLeftB.setPower(0);
        motorRightB.setPower(0);

        sleep((long)(pause*1000));

    }
    public void claw(double pause) throws InterruptedException {

        clawBody.setPosition(1);

        sleep(500);

        clawBody.setPosition(0);

        sleep((long) (pause * 1000));

    }

    public void setAngle(double theta, double pause) throws InterruptedException{

        tapeAngle.setPosition((3*theta) / 180);

        sleep((long) (pause * 1000));

    }

    public void tapeExt(double length, double pow, double pause) throws InterruptedException {

        double maxVel=2;

        tapeExt.setPower(-pow);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt.setPower(0);
        sleep((long) (pause * 1000));

    }

    public void tapePull(double length, double pow, double pause) throws InterruptedException{
        double maxVel=2;

        tapeExt.setPower(-pow);
        motorLeftA.setPower(-pow);
        motorRightA.setPower(-pow);
        motorLeftB.setPower(-pow);
        motorRightB.setPower(-pow);
        sleep((long) (1000 * (Math.abs(length / (maxVel * pow)))));

        tapeExt.setPower(0);
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
    public void blueRamp() throws InterruptedException {
        drive(1, -1, .5);
        turn(50, 1, .5);
        drive(2.4, -1, .5);
        turn(-92, 1, .5);
        drive(3.5, 1, 1);

        setAngle(60.0, .5);
        tapeExt(2.5,1,1.5);
        setAngle(0.0,2);
        tapePull(5,-1,.5);
    }

}