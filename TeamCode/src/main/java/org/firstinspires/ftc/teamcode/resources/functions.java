package org.firstinspires.ftc.teamcode.resources;

import android.graphics.Color;
import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.resources.external.ClosableVuforiaLocalizer;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.resources.constants.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;


/**
 * Created by dansm on 12/13/2017.
 */


public class functions {

    /*
    static public void moveUntilDistanceUltrasonic(ModernRoboticsI2cRangeSensor ultrasonicSensor, double targetDistance, double power, LinearOpMode linearOpMode){
        double distance = readAndFilterRangeSensorValues(linearOpMode);

        moveInAStraightLine(power);
        ElapsedTime mRuntime = new ElapsedTime();

        mRuntime.reset();

        while(distance < targetDistance && linearOpMode.opModeIsActive()){
            Log.d("JDRangeRear", "Time: "+ Double.toString(mRuntime.milliseconds()) + "Distance: " + Double.toString(distance));
            distance = readAndFilterRangeSensorValues(linearOpMode);
        }

        stopDriveMotors();
    }*/

    //scaling logic to use 4 fixed speeds as opposed to varying speeds to avoid jerks while driving
    static public double scaleInputFixedSpeed(double dVal) throws InterruptedException {
        int sign;

        if (dVal <= -MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS || dVal >= MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            sign = (int) (dVal / Math.abs(dVal));
        } else {
            sign = 1;
            return 0.0;
        }

        double result = Math.abs(dVal);

        if (result < 0.3) {
            result = 0.25;
        } else if (result < 0.5) {
            result = 0.4;
        } else if (result < 0.7) {
            result = 0.6;
        } else {
            result = 0.75;
        }
        return result * sign;
    }

    //Simple function to well scaleInput
    static public double scaleInput(double dVal) throws InterruptedException {
        double result = Math.pow(dVal, 3);
        if (result > 0.7) {
            result = 0.7;
        } else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

    //Same as scaling logic above, just using powers of 2 instead of 3
    static public double scaleInputPowerOf2(double dVal) {
        double sign = dVal / Math.abs(dVal);
        double result = (Math.pow(dVal, 2)) * sign;

        if (result > 0.7) {
            result = 0.7;
        } else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

    //Set the top, bottom, or both glyph grabber(s) to a position
    static public void setGrabber(double leftServoPosition, double rightServoPosition, GRABBERS grabbers) throws InterruptedException {
        switch (grabbers) {
            case BOTH_GRABBERS:
                glyphGrabberTL.setPosition(leftServoPosition);
                glyphGrabberTR.setPosition(rightServoPosition);
                glyphGrabberBL.setPosition(leftServoPosition);
                glyphGrabberBR.setPosition(rightServoPosition);

                break;

            case BOTTOM_GRABBER:
                glyphGrabberTL.setPosition(leftServoPosition);
                glyphGrabberTR.setPosition(rightServoPosition);

                break;
            case TOP_GRABBER:
                glyphGrabberBL.setPosition(leftServoPosition);
                glyphGrabberBR.setPosition(rightServoPosition);

                break;
        }
    }

    static public void moveArcade(Gamepad gamepad, LinearOpMode linearOpMode) throws InterruptedException {
        double r = Math.hypot(scaleInputFixedSpeed(-gamepad.left_stick_x), scaleInputFixedSpeed(gamepad.left_stick_y));
        double robotAngle = Math.atan2(scaleInputFixedSpeed(gamepad.left_stick_y), scaleInputFixedSpeed(-gamepad.left_stick_x)) - Math.PI / 4;

        double rightX = scaleInputFixedSpeed(-gamepad.right_stick_x);
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeftDriveMotor.setPower(v1);
        frontRightDriveMotor.setPower(-v2);
        backLeftDriveMotor.setPower(v3);
        backRightDriveMotor.setPower(-v4);
    }

    static public void moveArcadeFOD(Gamepad gamepad, FIELD_SIDE fieldSide, JDColor jdColor, LinearOpMode linearOpMode) throws InterruptedException {
        double r = Math.hypot(scaleInputFixedSpeed(-gamepad.left_stick_x), scaleInputFixedSpeed(gamepad.left_stick_y));

        double offset = 0;

        if (jdColor == JDColor.NONE) {
            offset = 0;
        } else if (fieldSide == FIELD_SIDE.RECOVERY_SIDE) {
            offset = Math.PI;
        } else if (fieldSide == FIELD_SIDE.JUDGE_SIDE && jdColor == JDColor.BLUE) {
            offset = -(Math.PI / 2);
        } else if (fieldSide == FIELD_SIDE.JUDGE_SIDE && jdColor == JDColor.RED) {
            offset = Math.PI / 2;
        }

        Orientation angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.RADIANS);

        //Note to self - first angle may have to be negative
        double robotAngle = (offset + (Math.atan2(scaleInputFixedSpeed(gamepad.left_stick_y), scaleInputFixedSpeed(-gamepad.left_stick_x)) - angles.firstAngle)) - Math.PI / 4;
        double rightX = scaleInputFixedSpeed(-gamepad.right_stick_x);

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeftDriveMotor.setPower(v1);
        frontRightDriveMotor.setPower(-v2);
        backLeftDriveMotor.setPower(v3);
        backRightDriveMotor.setPower(-v4);
    }

    static public void setJewelPosition(double jewelKnockerPosition, double jewelArmPosition) {
        jewelKnocker.setPosition(jewelKnockerPosition);
        jewelArm.setPosition(jewelArmPosition);
    }

    //open wide in Telep and open full wide in autonomous
    static public void initServos(boolean Teleop) throws InterruptedException {
        if (Teleop) {
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
            setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
            relicLinearServo.setPosition(0.3);
            relicRotationalServo.setPosition(1.0);
        } else {
            setGrabber(TOP_SERVO_GRABBER_INIT_POSITION[0], TOP_SERVO_GRABBER_INIT_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_INIT_POSITION[0], BOTTOM_SERVO_GRABBER_INIT_POSITION[1], BOTTOM_GRABBER);
            setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
            relicLinearServo.setPosition(0.3);
            relicRotationalServo.setPosition(1.0);
        }
    }

    static public void closeGrabber(GRABBERS grabbers) throws InterruptedException {
        switch (grabbers) {
            case BOTH_GRABBERS:
                setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);
                setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);

                break;
            case BOTTOM_GRABBER:
                setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);

                break;
            case TOP_GRABBER:
                setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);

                break;
        }
    }

    static public void openGrabber(GRABBERS grabbers) throws InterruptedException {
        switch (grabbers) {
            case BOTH_GRABBERS:
                setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);
                setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);

                break;

            case BOTTOM_GRABBER:
                setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);

                break;
            case TOP_GRABBER:
                setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);

                break;
        }
    }

    static public void openGrabberWide(GRABBERS grabbers) throws InterruptedException {
        switch (grabbers) {
            case BOTH_GRABBERS:
                setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);
                setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);

                break;

            case BOTTOM_GRABBER:
                setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);

                break;
            case TOP_GRABBER:
                setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);

                break;
        }
    }


    static public void moveInAStraightLine(double speed, boolean strafe) {
        if (!strafe) {
            frontLeftDriveMotor.setPower(-speed);
            frontRightDriveMotor.setPower(speed);
            backLeftDriveMotor.setPower(-speed);
            backRightDriveMotor.setPower(speed);
        } else {
            //Strafe; Positive speed is left, negative is right
            frontLeftDriveMotor.setPower(-speed);
            frontRightDriveMotor.setPower(-speed);
            backLeftDriveMotor.setPower(speed);
            backRightDriveMotor.setPower(speed);
        }
    }

    static public void moveInAStraightLine(double speed) {
        moveInAStraightLine(speed, false);
    }

    //Tank drive + strafing; Deprecated
    static public void move(double leftY, double rightY, double leftX, double rightX) throws InterruptedException {
        if (leftX >= STRAFING_LIMIT && rightX >= STRAFING_LIMIT || leftX <= -STRAFING_LIMIT && rightX <= -STRAFING_LIMIT) {
            //To strafe either left or right
            frontLeftDriveMotor.setPower(-leftX);
            frontRightDriveMotor.setPower(-leftX);
            backLeftDriveMotor.setPower(leftX);
            backRightDriveMotor.setPower(leftX);
        } else {
            //To move forwards/backwards/turn with tank drive controls
            frontLeftDriveMotor.setPower(leftY);
            frontRightDriveMotor.setPower(-rightY);
            backLeftDriveMotor.setPower(leftY);
            backRightDriveMotor.setPower(-rightY);
        }

    }

    static public void stopDriveMotors() {
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
    }

    static public void moveFirstLiftForTime(double speed, int milliseconds, LinearOpMode linearOpMode) {
        //Positive speed is up

        firstGlyphLift.setPower(-speed);
        linearOpMode.sleep(milliseconds);
        firstGlyphLift.setPower(0.0);
    }

    static public void moveSecondLiftForTime(double speed, int milliseconds, LinearOpMode linearOpMode) {
        //Positive speed is up

        secondGlyphLift.setPower(speed);
        linearOpMode.sleep(milliseconds);
        secondGlyphLift.setPower(0.0);
    }

    static public void moveForTime(double power, int milliseconds, LinearOpMode linearOpMode) {
        moveInAStraightLine(power);

        linearOpMode.sleep(milliseconds);
        stopDriveMotors();
    }

    static public void depositGlyph(LinearOpMode linearOpMode) throws InterruptedException {
        moveForTime(0.25, 500, linearOpMode);

        linearOpMode.sleep(250);

        openGrabberWide(BOTTOM_GRABBER);

        linearOpMode.sleep(250);

        moveForTime(-0.25, 250, linearOpMode);

        linearOpMode.sleep(250);

        moveForTime(0.25, 1000, linearOpMode);

        linearOpMode.sleep(250);

        moveForTime(-0.25, 250, linearOpMode);

        openGrabberWide(BOTTOM_GRABBER);

    }

    static public int firstLiftDirection = -1;
    static public int secondLiftDirection = -1;

    static ElapsedTime elapsedTimeForFirstGlyphLift = new ElapsedTime();
    static boolean sleepActive = false;

    static public void controlGlyphLifts(Gamepad gamepad, LinearOpMode linearOpMode) throws InterruptedException {
        if (firstLiftTopSwitch.getState() && gamepad.left_stick_y < -MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            firstGlyphLift.setPower(gamepad.left_stick_y);
        } else if (firstLiftBottomSwitch.getState() && gamepad.left_stick_y > MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            firstGlyphLift.setPower(gamepad.left_stick_y);
        } else {
            firstGlyphLift.setPower(0);
        }

        if (secondLiftTopSwitch.getState() && -gamepad.right_stick_y < -MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            secondGlyphLift.setPower(-gamepad.right_stick_y);
        } else if (secondLiftBottomSwitch.getState() && -gamepad.right_stick_y > MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            secondGlyphLift.setPower(-gamepad.right_stick_y);
        } else {
            secondGlyphLift.setPower(0);
        }
    }

    static public CryptoboxDetector initDogeCVForCryptobox(HardwareMap hMap, JDColor color) {

        CryptoboxDetector cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hMap.appContext, CameraViewDisplay.getInstance(), FRONT_FACING_CAMERA);

        cryptoboxDetector.downScaleFactor = 0.4;
        if (color == JDColor.RED) {
            cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
        } else if (color == JDColor.BLUE) {
            cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE;
        }

        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        cryptoboxDetector.rotateMat = true;


        cryptoboxDetector.enable();

        return cryptoboxDetector;
    }

    static public JewelDetector initDogeCVForJewel(HardwareMap hMap) {
        JewelDetector jewelDetector = new JewelDetector();
        jewelDetector.init(hMap.appContext, CameraViewDisplay.getInstance(), FRONT_FACING_CAMERA);
        jewelDetector.downScaleFactor = 0.4;

        jewelDetector.speed = JewelDetector.JewelDetectionSpeed.BALANCED;
        jewelDetector.rotateMat = true;

        jewelDetector.enable();

        return jewelDetector;
    }

    static public void readJewelsWithDogeCV(JewelDetector jewelDetector, LinearOpMode linearOpMode) {
        while (linearOpMode.opModeIsActive()) {
            ElapsedTime mRuntime = new ElapsedTime();

            linearOpMode.telemetry.addData("Status", "Run Time: " + mRuntime.toString());
            Log.d("JDJewel", "Run Time: " + mRuntime.toString());

            linearOpMode.telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
            Log.d("JDJewel", "Current Order: " + jewelDetector.getCurrentOrder().toString());

            linearOpMode.telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result
            Log.d("JDJewel", "Last Known Order: " + jewelDetector.getLastOrder().toString());

            if (linearOpMode.isStopRequested()) {
                jewelDetector.disable();
                break;
            }
        }
    }

    static public void lowerJewelArms(LinearOpMode linearOpMode) {
        jewelKnocker.setPosition(0.5);
        linearOpMode.sleep(200);
        jewelArm.setPosition(0);
        linearOpMode.sleep(1000);
    }

    static public void raiseJewelArms(LinearOpMode linearOpMode) {
        jewelArm.setPosition(0.9);
        linearOpMode.sleep(1000);
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(500);
    }

    static public void knockJewel(JDColor jewelColor, JDColor stoneColor, LinearOpMode linearOpMode) {
        if (jewelColor == JDColor.NONE) {
            //do nothing
        } else if (jewelColor == stoneColor) {
            kickOpposite(linearOpMode);
        } else if (jewelColor != stoneColor) {
            kickSame(linearOpMode);
        }
    }


    static public void kickOpposite(LinearOpMode linearOpMode) {
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(1500);
    }

    static public void kickSame(LinearOpMode linearOpMode) {
        jewelKnocker.setPosition(1);
        linearOpMode.sleep(1500);
    }


    static public void turn(int degrees, LinearOpMode linearOpMode, ElapsedTime globalRuntime) { //Positive degrees is counter-clockwise, negative degrees is clockwise
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Orientation angles;

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

        float originalZ = angles.firstAngle;
        float currentZ = angles.firstAngle;

        linearOpMode.telemetry.addData("Current Z", currentZ);
        linearOpMode.telemetry.addData("Original Z", originalZ);
        linearOpMode.telemetry.update();

        if (degrees > 0) {
            frontLeftDriveMotor.setPower(0.3);
            frontRightDriveMotor.setPower(0.3);
            backLeftDriveMotor.setPower(0.3);
            backRightDriveMotor.setPower(0.3);

            while ((!(currentZ >= degrees - 3) && (currentZ <= degrees + 3)) && linearOpMode.opModeIsActive() && globalRuntime.milliseconds() <= 3000) {
                angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                currentZ = angles.firstAngle;

                linearOpMode.telemetry.addData("Average Motor Speed", (frontLeftDriveMotor.getPower() + frontRightDriveMotor.getPower() + backLeftDriveMotor.getPower() + backRightDriveMotor.getPower()) / 4);
                linearOpMode.telemetry.addData("Current Z", currentZ);
                linearOpMode.telemetry.addData("Original Z", originalZ);
                linearOpMode.telemetry.update();
            }
        } else {
            frontLeftDriveMotor.setPower(-0.3);
            frontRightDriveMotor.setPower(-0.3);
            backLeftDriveMotor.setPower(-0.3);
            backRightDriveMotor.setPower(-0.3);

            while ((!(currentZ <= degrees + 3) && (currentZ >= degrees - 3)) && linearOpMode.opModeIsActive() && globalRuntime.milliseconds() <= 3000) {
                angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                currentZ = angles.firstAngle;

                linearOpMode.telemetry.addData("Average Motor Speed", (frontLeftDriveMotor.getPower() + frontRightDriveMotor.getPower() + backLeftDriveMotor.getPower() + backRightDriveMotor.getPower()) / 4);
                linearOpMode.telemetry.addData("Current Z", currentZ);
                linearOpMode.telemetry.addData("Original Z", originalZ);
                linearOpMode.telemetry.update();
            }
        }


        stopDriveMotors();
    }

    static public void doAllJewelStuff(JDColor stoneColor, LinearOpMode linearOpMode) {
        ArrayList<JDColor> listOfJewelColors = new ArrayList<constants.JDColor>();

        JewelDetectionOpenCV jewelVision = new JewelDetectionOpenCV();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        jewelVision.init(linearOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);

        // start the vision system
        jewelVision.enable();

        linearOpMode.sleep(100);

        ElapsedTime mRuntime = new ElapsedTime();

        mRuntime.reset();

        while (linearOpMode.opModeIsActive()) {
            linearOpMode.telemetry.addData("Jewel On Left", jewelVision.jewelOnLeft);
            linearOpMode.telemetry.addData("Time Elapsed", mRuntime.milliseconds());
            linearOpMode.telemetry.update();

            if (listOfJewelColors.size() < 5) {
                if (jewelVision.jewelOnLeft != constants.JDColor.NONE) {
                    listOfJewelColors.add(jewelVision.jewelOnLeft);
                }
            } else {
                break;
            }

            if (mRuntime.milliseconds() > 3000) {
                break;
            }
        }

        jewelVision.disable();

        int redJewelsFound = 0;
        int blueJewelsFound = 0;

        for (constants.JDColor color : listOfJewelColors) {
            if (color == constants.JDColor.RED) {
                redJewelsFound++;
            } else {
                blueJewelsFound++;
            }
        }

        lowerJewelArms(linearOpMode);

        int certainty = 0;

        constants.JDColor jewelOnRight = detectJewelColor(linearOpMode);

        //We assume we are on the blue side

        //Knock Jewel takes the color of the jewel on the RIGHT side, which is what we detect with the color sensor, but with OpenCV we detect the LEFT one
        if (blueJewelsFound >= 4 && jewelOnRight != JDColor.BLUE) {
            certainty = blueJewelsFound * 20;

            linearOpMode.telemetry.addData("Jewel On Left", "Blue");
            linearOpMode.telemetry.addData("Certainty", certainty);
            Log.d("JewelOnLeft", "Blue");
            Log.d("Certainty", Integer.toString(certainty));

            knockJewel(constants.JDColor.RED, stoneColor, linearOpMode);
        } else if (jewelOnRight == JDColor.RED) {
            if (!(redJewelsFound >= 4)) {
                knockJewel(constants.JDColor.RED, stoneColor, linearOpMode);
            }
        } else if (redJewelsFound >= 4 && jewelOnRight != JDColor.RED) {
            certainty = redJewelsFound * 20;

            linearOpMode.telemetry.addData("Jewel On Left", "Red");
            linearOpMode.telemetry.addData("Certainty", certainty);
            Log.d("JewelOnLeft", "Red");
            Log.d("Certainty", Integer.toString(certainty));

            knockJewel(constants.JDColor.BLUE, stoneColor, linearOpMode);
        } else if (jewelOnRight == JDColor.BLUE) {
            if (!(blueJewelsFound >= 4)) {
                knockJewel(constants.JDColor.BLUE, stoneColor, linearOpMode);
            }
        } else {
            linearOpMode.telemetry.addData("Jewel On Left", "Unclear");
            Log.d("JewelOnLeft", "Unknown");

            knockJewel(jewelOnRight, stoneColor, linearOpMode);
        }

        linearOpMode.telemetry.update();

        raiseJewelArms(linearOpMode);
    }


    static public JDColor detectJewelColor(LinearOpMode linearOpMode) {

        float hue = 0;

        JDColor jewelColorFound = JDColor.NONE;

        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        //read color for about 2 seconds
        while (jewelColorFound == JDColor.NONE && mRuntime.milliseconds() < 5) {

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;
            // sometimes it helps to multiply the raw RGB values with a scale factor
            // to amplify/attentuate the measured values.
            final double SCALE_FACTOR = 255;
            jewelColorSensor.enableLed(true);

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (jewelColorSensor.red() * SCALE_FACTOR), (int) (jewelColorSensor.green() * SCALE_FACTOR),
                    (int) (jewelColorSensor.blue() * SCALE_FACTOR), hsvValues);

            linearOpMode.telemetry.addData("hue", hsvValues[0]);
            linearOpMode.telemetry.addData("S", hsvValues[1]);
            linearOpMode.telemetry.addData("V", hsvValues[2]);
            linearOpMode.telemetry.update();

            hue = hsvValues[0];

            if (hue >= 190 && hue <= 235) {
                jewelColorFound = JDColor.BLUE;
            } else if (hue <= 15 || hue >= 350) {
                jewelColorFound = JDColor.RED;
            } else if (hue == 0) {
                jewelColorFound = JDColor.NONE;
            }

            Log.d("JDHue", "Hue " + Float.toString(hsvValues[0]));
        }

        Log.d("JDHue", "Color detected");

        linearOpMode.telemetry.addData("hue", Float.toString(hue));
        linearOpMode.telemetry.addData("Jewel Color", jewelColorFound.toString());
        linearOpMode.telemetry.update();
        jewelColorSensor.enableLed(false);
        return jewelColorFound;
    }

    static public void moveEncoders(int centimeters, double power, LinearOpMode linearOpMode) {
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = convertFromCMToTicks(centimeters);

        frontLeftDriveMotor.setTargetPosition(-ticks);
        frontRightDriveMotor.setTargetPosition(ticks);
        backLeftDriveMotor.setTargetPosition(-ticks);
        backRightDriveMotor.setTargetPosition(ticks);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDriveMotor.setPower(-power);
        frontRightDriveMotor.setPower(power);
        backLeftDriveMotor.setPower(-power);
        backRightDriveMotor.setPower(power);

        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        if (centimeters > 0) {
            while (backLeftDriveMotor.getCurrentPosition() <= ticks && backRightDriveMotor.getCurrentPosition() <= ticks && linearOpMode.opModeIsActive()) {
            }
        } else {
            while (backLeftDriveMotor.getCurrentPosition() >= ticks && backRightDriveMotor.getCurrentPosition() >= ticks && linearOpMode.opModeIsActive()) {
            }
        }

        stopDriveMotors();

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    static public ClosableVuforiaLocalizer initVuforia(HardwareMap hMap){

        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = " AZcIMlr/////AAAAGe1W/L9P20hXupxJsIH5bIMDl46JPwjrX2kI+L6+tigIG9bhthzvrEWVBni6g4Jkvs76N/hIT0bFun78pnNDqkG3ZP24XLj45VHA2rYKp8UDww/vfy8xrtvHxedihdX1A2vMWg8Ub8tLjBMgEAqcAYYUMwPRQfI61KQmXvAJBV79XtQughxCh/fbrtoux6WV6HHs8OydP7kPUaUU3f0z5ZOF/TUvcqFFotqnLg/KwXMxxrouRyDGCIbpbP7cYabiR7ShIGvrYoRKtbpwxS3WLSjjTd7ynvoidYipWZ60e6t+wUCzdXahS8g0veYuTQ+vwBqljhtLUWnCUjbJh2jocjxV9kLGgqlPFCmLHZyurYkX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);

        return vuforia;
    }

    static public RelicRecoveryVuMark getVumark(ClosableVuforiaLocalizer vuforia, LinearOpMode linearOpMode) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        //timer based fail safe logic
        ElapsedTime mRuntime = new ElapsedTime();

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //For debug purposes

        relicTrackables.activate();

        //try to read the vumark until we find a valid vumark or for 3 seconds
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && mRuntime.milliseconds() <= 3000 && linearOpMode.opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                linearOpMode.telemetry.addData("Vumark Found", vuMark.toString());
                linearOpMode.telemetry.update();
                break;
            }
            linearOpMode.telemetry.addData("Elapsed Time to find VuMark:", Double.toString(mRuntime.milliseconds()));
            linearOpMode.telemetry.update();
        }

        vuforia.close();

        linearOpMode.telemetry.addData("Found vumark", vuMark);

        Log.d("JDVumark", vuMark.toString());

        return vuMark;

    }


    static public void moveUntilCryptoWallUsingUltrasonic(double distanceToWall, RelicRecoveryVuMark vuMark, int color, LinearOpMode linearOpMode) {//Deprecated
        int targetColumn;

        if (vuMark == RelicRecoveryVuMark.LEFT && color == RED) {
            targetColumn = 1;
        } else if (vuMark == RelicRecoveryVuMark.CENTER && color == RED) {
            targetColumn = 2;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT && color == RED) {
            targetColumn = 3;
        } else if (vuMark == RelicRecoveryVuMark.LEFT && color == BLUE) {
            targetColumn = 1;
        } else if (vuMark == RelicRecoveryVuMark.CENTER && color == BLUE) {
            targetColumn = 2;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT && color == BLUE) {
            targetColumn = 3;
        } else {
            targetColumn = 1;
        }

        int columnsPassed = 0;


        double distanceToCryptoBoxWall = distanceToWall - 5;


        if (color == RED) {
            frontLeftDriveMotor.setPower(0.2);
            frontRightDriveMotor.setPower(-0.2);
            backLeftDriveMotor.setPower(0.2);
            backRightDriveMotor.setPower(-0.2);
        } else if (color == BLUE) {
            frontLeftDriveMotor.setPower(-0.2);
            frontRightDriveMotor.setPower(0.2);
            backLeftDriveMotor.setPower(-0.2);
            backRightDriveMotor.setPower(0.2);
        }

        frontLeftDriveMotor.setPower(0.2);
        frontRightDriveMotor.setPower(-0.2);
        backLeftDriveMotor.setPower(0.2);
        backRightDriveMotor.setPower(-0.2);


        while (linearOpMode.opModeIsActive()) {
            if (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall) {
                columnsPassed++;

                while (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall && linearOpMode.opModeIsActive()) {
                }
            }

            if (columnsPassed >= targetColumn) {
                stopDriveMotors();
                break;
            }

            linearOpMode.telemetry.addData("Distance to Wall", distanceToWall);
            linearOpMode.telemetry.addData("Distance to Crypto Box Wall", distanceToCryptoBoxWall);
            linearOpMode.telemetry.addData("Centimeters from Object", sideRangeSensor.cmUltrasonic());
            linearOpMode.telemetry.addData("Columns Passed", columnsPassed);
            linearOpMode.telemetry.update();
        }
    }

    //discard any accidental bad reading from the sensor
    //break out of the loop if unable to read good sensor data within 200ms
    static public double readAndFilterRangeSensorValues(ModernRoboticsI2cRangeSensor ultrasonicSensor, LinearOpMode linearOpMode) {
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        double distance = ultrasonicSensor.cmUltrasonic();
        while ((distance == 255 || distance == 0) && linearOpMode.opModeIsActive()) {
            distance = ultrasonicSensor.cmUltrasonic();
        }
        return distance;
    }

    static public void moveUntilCryptoWallUsingUltrasonicv2(double startDistance, RelicRecoveryVuMark vuMark, JDColor allianceColor, FIELD_SIDE fieldSide, LinearOpMode linearOpMode, ElapsedTime globalRuntime) { //Deprecated
        int targetColumn = targetCryptoboxColumn(vuMark, allianceColor, fieldSide);
        int cryptoWallMinVal = 5;

        int columnsPassed = 0;
        boolean firstTime = true;

        double distanceToCrypto = startDistance - cryptoWallMinVal;

        double motorSpeedRed = 0.20;
        double motorSpeedBlue = 0.20;

        if (allianceColor == JDColor.RED) {
            frontLeftDriveMotor.setPower(motorSpeedRed);
            frontRightDriveMotor.setPower(-motorSpeedRed);
            backLeftDriveMotor.setPower(motorSpeedRed);
            backRightDriveMotor.setPower(-motorSpeedRed);
        } else if (allianceColor == JDColor.BLUE) {
            frontLeftDriveMotor.setPower(-motorSpeedBlue);
            frontRightDriveMotor.setPower(motorSpeedBlue);
            backLeftDriveMotor.setPower(-motorSpeedBlue);
            backRightDriveMotor.setPower(motorSpeedBlue);
        }

        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        String msg = "";

        double distance = readAndFilterRangeSensorValues(sideRangeSensor, linearOpMode);

        boolean firstTimeIncrementingColumnPassed = true;

        while (linearOpMode.opModeIsActive() && globalRuntime.milliseconds() < 2000) {

            distance = readAndFilterRangeSensorValues(sideRangeSensor, linearOpMode);

            // move robot past each crypto column
            //as soon as target column is seen break out of the loop and stopDriveMotors.
            //sometimes the distance is less than the minimum distance of 5, so check if less than 5 or less than 4
            while ((distance <= distanceToCrypto || distance <= distanceToCrypto - 1) && linearOpMode.opModeIsActive()) {

                msg = Double.toString(mRuntime.milliseconds()) + ": "
                        + Double.toString(distance)
                        + " Column: " + Integer.toString(columnsPassed)
                        + " CryptoDistance: " + distanceToCrypto;
                linearOpMode.telemetry.addData("range:", msg);
                linearOpMode.telemetry.update();
                Log.d("JDRange", msg);

                //column increased only the first time when there is a change in distance

                if (firstTime && !(targetColumn == 3 && firstTimeIncrementingColumnPassed)) {
                    columnsPassed++;
                    firstTime = false;
                    Log.d("JDRange", "Incremented columnsPassed");
                } else if (targetColumn == 3 && firstTimeIncrementingColumnPassed) {
                    Log.d("JDRange", "Sleep 100 Milliseconds");
                    linearOpMode.sleep(100);
                    firstTimeIncrementingColumnPassed = false;
                } else if (firstTime) {
                    columnsPassed++;
                    firstTime = false;
                    Log.d("JDRange", "1, 2 Incremented");
                }


                //target column is reached break out the while loop reading range sensor data
                //if last column let it stopDriveMotors at the 4th column, else let it stopDriveMotors at the exact column
                if (columnsPassed >= targetColumn || mRuntime.milliseconds() >= MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS) {
                    break;
                }

                distance = readAndFilterRangeSensorValues(sideRangeSensor, linearOpMode);
            }

            //reset the first time for the next columns
            if (!firstTime) {
                firstTime = true;

            }

            //adjust the minimum distance based on the new reading as the robot might have drifted
            distanceToCrypto = distance - cryptoWallMinVal;


            //target column is reached or if estimate time elapsed, got to break and stopDriveMotors the robot
            //if last column let it stopDriveMotors at the 4th column, else let it stopDriveMotors at the exact column
            if (columnsPassed >= targetColumn || mRuntime.milliseconds() >= MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS) {
                msg = Double.toString(mRuntime.milliseconds()) + ": "
                        + Double.toString(distance)
                        + " Column: " + Integer.toString(columnsPassed)
                        + " CryptoDistance: " + distanceToCrypto;
                linearOpMode.telemetry.addData("range:", msg);
                linearOpMode.telemetry.update();
                Log.d("JDRange", msg);
                if (allianceColor == JDColor.RED) {
                    if (targetColumn == 3) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 1200 MS");
                        moveForTime(-motorSpeedRed, 1200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved backwards for 1200 MS");
                    } else if (targetColumn == 2) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 200 MS");
                        moveForTime(-motorSpeedRed, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved backwards for 200 MS");
                    } /*else if (targetColumn == 1) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 200 MS");
                        moveForTime(-motorSpeedRed, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ":Moved backwards for 200 MS");
                    }*/

                    break;
                } else if (allianceColor == JDColor.BLUE) {
                    if (targetColumn == 3) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 1200 MS");
                        moveForTime(motorSpeedBlue, 1200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved forwards for 1200 MS");
                    } else if (targetColumn == 2) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 200 MS");
                        moveForTime(motorSpeedBlue, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved forwards for 200 MS");
                    } /*else if (targetColumn == 1) {
                        stopDriveMotors();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 200 MS");
                        moveForTime(motorSpeedBlue, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ":Moved forwards for 200 MS");
                    }*/

                    break;
                }
            }

            msg = Double.toString(mRuntime.milliseconds()) + ": "
                    + Double.toString(distance)
                    + " Column: " + Integer.toString(columnsPassed)
                    + " CryptoDistance: " + distanceToCrypto;
            linearOpMode.telemetry.addData("range:", msg);
            linearOpMode.telemetry.update();
            Log.d("JDRange", msg);
        }

        stopDriveMotors();

    }

    static private int targetCryptoboxColumn(RelicRecoveryVuMark vuMark, JDColor allianceColor, FIELD_SIDE fieldSide) {
        int targetColumn = 1;

        switch (vuMark) {
            case LEFT:
                if (allianceColor == JDColor.RED) {
                    targetColumn = 3;
                } else if (allianceColor == JDColor.BLUE) {
                    targetColumn = 1;
                }
                break;

            case CENTER:
                targetColumn = 2;
                break;

            case RIGHT:
                if (allianceColor == JDColor.RED) {
                    targetColumn = 1;
                } else if (allianceColor == JDColor.BLUE) {
                    targetColumn = 3;
                }
                break;

            default:
                targetColumn = 1;
                break;
        }

        return targetColumn;
    }

    static public void moveToCryptoColumnEncoders(RelicRecoveryVuMark vuMark, JDColor allianceColor, FIELD_SIDE fieldSide, LinearOpMode linearOpMode) {
        int distanceToTravel = 0; //It should always be set to something other than 0, this is just so the compiler doesn't yell at me

        //Defaults to smallest value as that is closest column so > chance of success
        if (allianceColor == JDColor.RED && fieldSide == FIELD_SIDE.JUDGE_SIDE) {
            switch (vuMark) {
                case LEFT:
                    distanceToTravel = -42;
                    break;
                case CENTER:
                    distanceToTravel = -9;
                    break;
                case RIGHT:
                    distanceToTravel = 3;
                    break;
                default:
                    distanceToTravel = 3;
            }
        } else if (allianceColor == JDColor.BLUE && fieldSide == FIELD_SIDE.JUDGE_SIDE) {
            switch (vuMark) {
                case LEFT:
                    distanceToTravel = -3;
                    break;
                case CENTER:
                    distanceToTravel = -6;
                    break;
                case RIGHT:
                    distanceToTravel = -20;
                    break;
                default:
                    distanceToTravel = -3;
            }
        } else if (allianceColor == JDColor.RED && fieldSide == FIELD_SIDE.RECOVERY_SIDE) {
            switch (vuMark) {
                case LEFT:
                    distanceToTravel = -16;
                    break;
                case CENTER:
                    distanceToTravel = -4;
                    break;
                case RIGHT:
                    distanceToTravel = 4;
                    break;
                default:
                    distanceToTravel = 4;
            }
        } else if (allianceColor == JDColor.BLUE && fieldSide == FIELD_SIDE.RECOVERY_SIDE) {
            switch (vuMark) {
                case LEFT:
                    distanceToTravel = -4;
                    break;
                case CENTER:
                    distanceToTravel = 4;
                    break;
                case RIGHT:
                    distanceToTravel = 16;
                    break;
                default:
                    distanceToTravel = -4;
            }
        }


        if (distanceToTravel < 0) {
            moveEncoders(distanceToTravel, -0.25, linearOpMode);
        } else if (distanceToTravel > 0) {
            moveEncoders(distanceToTravel, 0.25, linearOpMode);
        }


    }


    //AVT Algorithm to filter range sensor values and return the sampled distance
    //calculate average value
    //AVT algorithm stands for Antonyan Vardan Transform and its implementation explained below.
    //Collect n samples of data
    //Calculate the standard deviation and average value
    //Drop any data that is greater or less than average ± one standard deviation
    //Calculate average value of remaining data

    static public double filterRangeSensorValues(double[] rangeSensorValues) {

        //calculate average
        double sum = 0;
        for (int i = 0; i < rangeSensorValues.length; i++)
            sum = sum + rangeSensorValues[i];
        double average = sum / rangeSensorValues.length;

        //calculate standard deviation
        double sd = 0;
        for (int i = 0; i < rangeSensorValues.length; i++) {
            sd = sd + Math.pow((rangeSensorValues[i] - average), 2);
        }
        double standardDeviation = Math.sqrt(sd / rangeSensorValues.length);

        //Drop any data that is greater or less than average ± one standard deviation
        int newlength = 0;
        double newSum = 0;
        for (int i = 0; i < rangeSensorValues.length; i++) {
            if (!(rangeSensorValues[i] > average + standardDeviation || rangeSensorValues[i] < average - standardDeviation)) {
                newlength++;
                newSum = newSum + rangeSensorValues[i];
            }
        }

        //distance is average value of remaining data
        return newSum / newlength;
    }

    static public void moveToDistanceUltrasonic(ModernRoboticsI2cRangeSensor rangeSensor, int centimeters, double power, DIRECTION direction, int timeLimit, LinearOpMode linearOpMode, ElapsedTime globalRuntime) {

        Log.d("JDTime", Double.toString(globalRuntime.milliseconds()));

        if (direction == DIRECTION.MOVING_TOWARDS_OBJECT) {
            moveInAStraightLine(power);
            while (readAndFilterRangeSensorValues(rangeSensor, linearOpMode) > centimeters && linearOpMode.opModeIsActive() && globalRuntime.milliseconds() <= timeLimit) {
                linearOpMode.telemetry.addData("Distance", rangeSensor.cmUltrasonic());
                linearOpMode.telemetry.update();
                Log.d("JDDistance", Double.toString(readAndFilterRangeSensorValues(rangeSensor, linearOpMode)));
            }
        }
        if (direction == DIRECTION.MOVING_AWAY_FROM_OBJECT) {
            moveInAStraightLine(power);
            while (readAndFilterRangeSensorValues(rangeSensor, linearOpMode) < centimeters && linearOpMode.opModeIsActive() && globalRuntime.milliseconds() <= timeLimit) {
                Log.d("JDDistance", Double.toString(readAndFilterRangeSensorValues(rangeSensor, linearOpMode)));
                linearOpMode.telemetry.addData("Distance", rangeSensor.cmUltrasonic());
                linearOpMode.telemetry.update();
            }
        }
        Log.d("JDTime", Double.toString(globalRuntime.milliseconds()));
        stopDriveMotors();
    }


    static public void turnPID(double degrees, boolean gettingCoeffecientsThroughUdp) {
        PID pidClass = new PID();

        Orientation angles;

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

        pidClass.setCoeffecients(0.015, 0.0, 0.002);

        double motorSpeed;

        double allowableError = 0.3;

        while (!(angles.firstAngle > degrees - allowableError && angles.firstAngle < degrees + allowableError)) {
            angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

            if (gettingCoeffecientsThroughUdp) {
                motorSpeed = pidClass.calculateOutput(degrees, angles.firstAngle, true);
            } else {
                motorSpeed = pidClass.calculateOutput(degrees, angles.firstAngle);
            }

            frontLeftDriveMotor.setPower(motorSpeed);
            frontRightDriveMotor.setPower(motorSpeed);
            backLeftDriveMotor.setPower(motorSpeed);
            backRightDriveMotor.setPower(motorSpeed);

            angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        }
        if (gettingCoeffecientsThroughUdp) {
            Log.d("JDPID", "Shutting down Udp receiver");
            pidClass.pidUdpReceiver.shutdown();
        }

        stopDriveMotors();
    }

    static public void turnPID(double degrees) {
        turnPID(degrees, false);
    }


    static public void moveToDistanceUltrasonicPID(ModernRoboticsI2cRangeSensor rangeSensor, int centimeters, LinearOpMode linearOpMode, boolean gettingCoeffecientsThroughUdp) {
        PID pidClass = new PID();

        pidClass.setCoeffecients(0.01, 0, 0.002);

        double distance = readAndFilterRangeSensorValues(rangeSensor, linearOpMode);

        double motorSpeed;

        while (!(distance > centimeters - 1 && distance < centimeters + 1) && linearOpMode.opModeIsActive()) {
            distance = readAndFilterRangeSensorValues(rangeSensor, linearOpMode);

            if (gettingCoeffecientsThroughUdp) {
                motorSpeed = pidClass.calculateOutput(centimeters, distance, true);
            } else {
                motorSpeed = pidClass.calculateOutput(centimeters, distance);
            }

            moveInAStraightLine(-motorSpeed);

            linearOpMode.telemetry.addData("Distance", distance);

            linearOpMode.telemetry.addData("Front Left Encoder", frontLeftDriveMotor.getCurrentPosition());
            linearOpMode.telemetry.addData("Front Right Encoder", frontRightDriveMotor.getCurrentPosition());
            linearOpMode.telemetry.addData("Back Left Encoder", backLeftDriveMotor.getCurrentPosition());
            linearOpMode.telemetry.addData("Back Right Encoder", backRightDriveMotor.getCurrentPosition());

            linearOpMode.telemetry.update();
        }

        if (gettingCoeffecientsThroughUdp) {
            Log.d("JDPID", "Shutting down Udp receiver");
            pidClass.pidUdpReceiver.shutdown();
        }

        stopDriveMotors();
    }

    static public void moveToDistanceUltrasonicPID(ModernRoboticsI2cRangeSensor rangeSensor, int centimeters, LinearOpMode linearOpMode) {
        moveToDistanceUltrasonicPID(rangeSensor, centimeters, linearOpMode, false);
    }

    static public double convertFromTicksToCM(int ticks){
        return (2.54 * ticks) / (7 * Math.PI);  //Convert the amount of ticks to centimeters
    }

    static public int convertFromCMToTicks(double cm){
        return (int) ((((((Math.PI * 4) / 16) * 112) * cm) / 4) / 2.54); //Convert the amount of centimeters to ticks
    }

    static public void moveWithMotionProfiling(double centimeters, String csvFile, LinearOpMode linearOpMode){
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double centimetersTraveled = 0;

        MotionProfiling motionProfilingClass = new MotionProfiling();

        motionProfilingClass.setCoeffecients(, 0.001); //Kv should be ~ 1/max velocity
        if (motionProfilingClass.readMotionProfileFile("")) {
            while (linearOpMode.opModeIsActive()) {
                centimetersTraveled = convertFromTicksToCM(((backLeftDriveMotor.getCurrentPosition() + backRightDriveMotor.getCurrentPosition()) / 2));

                if(centimetersTraveled > centimeters - 0.1 && centimetersTraveled < centimeters + 0.1){
                    break;
                }

                moveInAStraightLine(motionProfilingClass.calculatePower(centimeters, centimetersTraveled));


            }
            stopDriveMotors();
        }
        else{
            moveEncoders((int) centimeters, 0.3, linearOpMode);
        }

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
