package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class FourWheelDrive extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.

    ColorSensor colorSensor;

    DcMotor motorRightA;
    DcMotor motorRightB;
    DcMotor motorLeftA;
    DcMotor motorLeftB;
    DcMotor tapeMain;
    DcMotor tapeExt2;
    Servo dropper;
    Servo trigger1;
    Servo trigger2;
    Servo tapeAngle;


    final static double tapeDelta = .0025;
    final static double triggerDelta = .0025;
    double leftPosition = 0;
    double rightPosition = 1;
    double tapePosition = 0;
    boolean triggerLeft = false;
    boolean triggerRight = false;


    /**
     * Constructor
     */
    public FourWheelDrive() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorRightA = hardwareMap.dcMotor.get("motor_fr");
        motorRightB = hardwareMap.dcMotor.get("motor_br");
        motorLeftA = hardwareMap.dcMotor.get("motor_fl");
        motorLeftB = hardwareMap.dcMotor.get("motor_bl");
        tapeMain = hardwareMap.dcMotor.get("motor_ext1");
        tapeExt2 = hardwareMap.dcMotor.get("motor_ext2");
        motorLeftA.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);

        dropper = hardwareMap.servo.get("servo_1");
        trigger1 = hardwareMap.servo.get("servo_2");
        trigger2 = hardwareMap.servo.get("servo_3");
        tapeAngle = hardwareMap.servo.get("servo_4");


        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        trigger1.setPosition(leftPosition);
        trigger2.setPosition(rightPosition);
        dropper.setPosition(1);
        tapeAngle.setPosition(.45);

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {



		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float throttle = gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;


        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // sets value to motors
        motorRightA.setPower(right);
        motorRightB.setPower(right);
        motorLeftA.setPower(left);
        motorLeftB.setPower(left);
        colorSensor.enableLed(true);

        if (gamepad2.dpad_up) {
            tapePosition += tapeDelta;
        }
        if (gamepad2.dpad_down) {
            tapePosition -= tapeDelta;
        }

        tapePosition = Range.clip(tapePosition, 0, 1);

        tapeAngle.setPosition(tapePosition);


        tapeMain.setPower(gamepad2.right_stick_y * -1);
        if (gamepad2.right_stick_y > 0) {
            tapeExt2.setPower(gamepad2.right_stick_y * .1);
        }
        if (gamepad2.right_stick_y < 0) {
            tapeExt2.setPower(gamepad2.right_stick_y * .2);
        }
        if (gamepad2.right_stick_y == 0) {
            tapeExt2.setPower(gamepad2.right_stick_y * 0);
        }



        if (gamepad2.right_bumper) {
            dropper.setPosition(0);
        }

        if (gamepad2.left_bumper) {
            dropper.setPosition(1);
        }

        if(gamepad1.b){
            triggerLeft = true;
            triggerRight = false;
        }

        if(gamepad1.x){
            triggerLeft = false;
            triggerRight = true;
        }

        if(triggerLeft==true){
            if(gamepad1.left_bumper){
                leftPosition -= triggerDelta;
            }
            if(gamepad1.right_bumper){
                leftPosition += triggerDelta;
            }
        }

        if(triggerRight==true){
            if(gamepad1.left_bumper){
                rightPosition += triggerDelta;
            }
            if(gamepad1.right_bumper){
                rightPosition -= triggerDelta;
            }
        }
//
        leftPosition = Range.clip(leftPosition, 0, 1);
        rightPosition = Range.clip(rightPosition, 0, 1);

        trigger1.setPosition(leftPosition);
        trigger2.setPosition(rightPosition);


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;




        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Red:", colorSensor.red());
        telemetry.addData("Blue:", colorSensor.blue());
        telemetry.addData("Green:", colorSensor.green());
        telemetry.addData("Scale:", scaleFront(right));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * Scales the right and left values utilizing the Howerton-Chance Algorithm
     */

    double scaleInput(double dVal) {

        if (gamepad1.a) {
            if (dVal > 0) {
                dVal = .4 * (Math.pow(2.71828, (dVal * .4)) - 1);
            }
            if (dVal < 0) {
                dVal = -.4 * (Math.pow(2.71828, (dVal * -.4)) - 1);
            }
        } else {
            if (dVal > 0) {
                dVal = .8 * (Math.pow(2.71828, (dVal * .4)) - 1);
            }
            if (dVal < 0) {
                dVal = -.8 * (Math.pow(2.71828, (dVal * -.4)) - 1);
            }
        }

        return dVal;
    }


    double scaleFront(double pow) {
        double z = (-151.727-(Math.sqrt(Math.pow(151.727,2)-(4*(-93.939)*(92.9333-(.8181*pow))))))/(2*(-93.939));
        if (pow < .2) {
            pow*=.818181;
        }
        if (pow >= .2) {
            pow = 1/((-151.727-(Math.sqrt(Math.pow(151.727,2)-(4*(-93.939)*(92.9333-(.8181*z))))))/(2*(-93.939)));
        }

        return pow;
    }
}