package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTarget;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.constants.*;
import static org.firstinspires.ftc.teamcode.hardware.*;



/**
 * Created by dansm on 12/13/2017.
 */


public class functions{

    static public void moveDistanceUltrasonic(ModernRoboticsI2cRangeSensor ultrasonicSensor, double targetDistance, double power, LinearOpMode linearOpMode){
        double distance = readAndFilterRangeSensor(linearOpMode);

        moveInAStraightLine(power);
        ElapsedTime mRuntime = new ElapsedTime();

        mRuntime.reset();

        while(distance < targetDistance && linearOpMode.opModeIsActive()){
            Log.d("JDRangeRear", "Time: "+ Double.toString(mRuntime.milliseconds()) + "Distance: " + Double.toString(distance));
            distance = readAndFilterRangeSensor(linearOpMode);
        }

        stop();
    }

    //scaling logic 2 to use 4 fixed speeds as opposed to varying speeds to avoid jerks while driving
    static public double scaleInputFixedSpeed(double dVal) throws InterruptedException{
        int sign;

        if(dVal <= -MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS || dVal >= MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            sign = (int) (dVal / Math.abs(dVal));
        }
        else{
            sign=1;
            return 0.0;
        }

        double result = Math.abs(dVal);

        if(result < 0.3){
            result = 0.25;
        }

        else if(result < 0.5 ){
            result = 0.4;
        }
        else if(result < 0.7){
            result = 0.6;
        }
        else{
            result = 0.75;
        }
        return result*sign ;
    }

    static public double scaleInput(double dVal) throws InterruptedException{
        double result = Math.pow(dVal, 3);
        if (result > 0.7){
            result = 0.7;
        }
        else if (result < -0.7){
            result = -0.7;
        }
        return result;
    }

    static public double scaleInputPowerOf2(double dVal){
        double sign = dVal/ Math.abs(dVal);
        double result = (Math.pow(dVal, 2)) * sign;

        if (result > 0.7){
            result = 0.7;
        }
        else if (result < -0.7){
            result = -0.7;
        }
        return result;
    }

    static public void setGrabber(double leftServoPosition, double rightServoPosition, int grabbers) throws InterruptedException{
        if(grabbers == BOTH_GRABBERS){
            glyphGrabberTL.setPosition(leftServoPosition);
            glyphGrabberTR.setPosition(rightServoPosition);
            glyphGrabberBL.setPosition(leftServoPosition);
            glyphGrabberBR.setPosition(rightServoPosition);
        }
        else if(grabbers == BOTTOM_GRABBER){
            glyphGrabberBL.setPosition(leftServoPosition);
            glyphGrabberBR.setPosition(rightServoPosition);
        }
        else if(grabbers == TOP_GRABBER){
            glyphGrabberTL.setPosition(leftServoPosition);
            glyphGrabberTR.setPosition(rightServoPosition);
        }
    }

    static public void moveArcade(Gamepad gamepad) throws InterruptedException{
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

    static public void setJewelPosition(double jewelKnockerPosition, double jewelArmPosition){
        jewelKnocker.setPosition(jewelKnockerPosition);
        jewelArm.setPosition(jewelArmPosition);
    }

    //open wide in Telep and open full wide in autonomous
    static public void initServos(boolean Teleop) throws InterruptedException{
        if(Teleop){
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
            setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
        }
        else {
            setGrabber(TOP_SERVO_GRABBER_INIT_POSITION[0], TOP_SERVO_GRABBER_INIT_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_INIT_POSITION[0], BOTTOM_SERVO_GRABBER_INIT_POSITION[1], BOTTOM_GRABBER);
            setJewelPosition(JEWEL_KNOCKER_INIT_POSITION, JEWEL_ARM_INIT_POSITION);
        }
    }

    static public void closeGrabber(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_CLOSE_POSITION[0], TOP_SERVO_GRABBER_CLOSE_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_CLOSE_POSITION[0], BOTTOM_SERVO_GRABBER_CLOSE_POSITION[1], BOTTOM_GRABBER);
        }
    }

    static public void openGrabber(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_OPEN_POSITION[0], TOP_SERVO_GRABBER_OPEN_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_OPEN_POSITION[1], BOTTOM_GRABBER);
        }
    }

    static public void openGrabberWide(int grabber) throws InterruptedException{
        if (grabber == BOTH_GRABBERS){
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
        }else if ( grabber == TOP_GRABBER){
            setGrabber(TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[0], TOP_SERVO_GRABBER_WIDE_OPEN_POSITION[1], TOP_GRABBER);

        }else if (grabber == BOTTOM_GRABBER){
            setGrabber(BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[0], BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION[1], BOTTOM_GRABBER);
        }
    }

    static public void moveInAStraightLine(double speed, boolean strafe){
        if(!strafe){
            frontLeftDriveMotor.setPower(-speed);
            frontRightDriveMotor.setPower(speed);
            backLeftDriveMotor.setPower(-speed);
            backRightDriveMotor.setPower(speed);
        }
        else{
            //Strafe; Positive speed is left, negative is right
            frontLeftDriveMotor.setPower(-speed);
            frontRightDriveMotor.setPower(-speed);
            backLeftDriveMotor.setPower(speed);
            backRightDriveMotor.setPower(speed);
        }
    }

    static public void moveInAStraightLine(double speed){
        moveInAStraightLine(speed, false);
    }

    static public void move(double leftY, double rightY, double leftX, double rightX) throws InterruptedException{
        if(leftX >= STRAFING_LIMIT && rightX >= STRAFING_LIMIT || leftX <= -STRAFING_LIMIT && rightX <= -STRAFING_LIMIT){
            //To strafe either left or right
            frontLeftDriveMotor.setPower(-leftX);
            frontRightDriveMotor.setPower(-leftX);
            backLeftDriveMotor.setPower(leftX);
            backRightDriveMotor.setPower(leftX);
        }

        else{
            //To move forwards/backwards/turn with tank drive controls
            frontLeftDriveMotor.setPower(leftY);
            frontRightDriveMotor.setPower(-rightY);
            backLeftDriveMotor.setPower(leftY);
            backRightDriveMotor.setPower(-rightY);
        }

    }

    static public void stop(){
        frontLeftDriveMotor.setPower(0);
        frontRightDriveMotor.setPower(0);
        backLeftDriveMotor.setPower(0);
        backRightDriveMotor.setPower(0);
    }

    static public void moveLiftForTime(double speed, int milliseconds, LinearOpMode linearOpMode){
        //Positive speed is up

        firstGlyphLift.setPower(-speed);
        linearOpMode.sleep(milliseconds);
        firstGlyphLift.setPower(0.0);
    }

    static public void moveSecondLiftForTime(double speed, int milliseconds, LinearOpMode linearOpMode){
        //Positive speed is up

        secondGlyphLift.setPower(speed);
        linearOpMode.sleep(milliseconds);
        secondGlyphLift.setPower(0.0);
    }

    static public void moveForTime(double power, int milliseconds, LinearOpMode linearOpMode){
        moveInAStraightLine(power);

        linearOpMode.sleep(milliseconds);
        stop();
    }

    static public void depositGlyph(LinearOpMode linearOpMode) throws InterruptedException{
        moveForTime(0.3, 450, linearOpMode);

        linearOpMode.sleep(250);

        openGrabberWide(BOTTOM_GRABBER);

        linearOpMode.sleep(250);

        moveForTime(0.3, 650, linearOpMode);

        linearOpMode.sleep(250);

        moveForTime(-0.3, 300, linearOpMode);

    }

    static public int firstLiftDirection = -1;
    static public int secondLiftDirection = -1;

    static public void firstLift(Gamepad gamepad2, LinearOpMode linearOpMode) throws InterruptedException{
        if(!firstLiftSwitch.getState() && firstLiftDirection == UP){
            linearOpMode.telemetry.addData("First Lift", "Top Limit Reached - Move Down");
            linearOpMode.telemetry.update();

            if (gamepad2.left_stick_y > LIFT_TWITCH_THRESHOLD){
                //Move down at a slow speed as gravity is pulling it down
                firstGlyphLift.setPower(0.4);
                //allows sensor to move away from the magnet
                linearOpMode.sleep(450);
                firstLiftDirection = DOWN;
            }
            else{
                //Don't allow to move up any further
                firstGlyphLift.setPower(0);
                firstLiftDirection = UP;
            }
        }
        else if(!firstLiftSwitch.getState() && firstLiftDirection == DOWN){
            linearOpMode.telemetry.addData("First Lift", "Bottom Limit Reached - Move Up");
            linearOpMode.telemetry.update();

            if(gamepad2.left_stick_y < -LIFT_TWITCH_THRESHOLD){
                firstGlyphLift.setPower(-0.8);
                //allows sensor to move away from the magnet
                linearOpMode.sleep(600);
                firstLiftDirection = UP;
            }
            else {
                //Don't allow to move down any further
                firstGlyphLift.setPower(0);
                firstLiftDirection = DOWN;
            }
        }
        else{
            if(gamepad2.left_stick_y < -LIFT_TWITCH_THRESHOLD){
                firstGlyphLift.setPower(-0.8); //going up is steady speed of 0.8
                firstLiftDirection = UP;
            }
            else if(gamepad2.left_stick_y > LIFT_TWITCH_THRESHOLD){
                firstGlyphLift.setPower(gamepad2.left_stick_y/2);
                firstLiftDirection = DOWN;
            }
            else{
                firstGlyphLift.setPower(0);
            }

            linearOpMode.telemetry.addData("First Lift", "Can move freely", true);
            linearOpMode.telemetry.update();
        }
    }

    static public void secondLift(Gamepad gamepad2, LinearOpMode linearOpMode) throws InterruptedException{
        if(!secondLiftSwitch.getState() && secondLiftDirection == UP){
            linearOpMode.telemetry.addData("Second Lift", "Top Limit Reached - Move Down");
            linearOpMode.telemetry.update();

            if (gamepad2.right_stick_y > LIFT_TWITCH_THRESHOLD){
                secondGlyphLift.setPower(-0.3);
                linearOpMode.sleep(500);
                secondLiftDirection = DOWN;
            }
            else{
                secondGlyphLift.setPower(0);
                secondLiftDirection = UP;
            }
        }
        else if(!secondLiftSwitch.getState() && secondLiftDirection == DOWN){
            linearOpMode.telemetry.addData("Second Lift", "Bottom Limit Reached - Move Up");
            linearOpMode.telemetry.update();

            if(gamepad2.right_stick_y < -LIFT_TWITCH_THRESHOLD){
                secondGlyphLift.setPower(0.5);
                linearOpMode.sleep(500);
                secondLiftDirection = UP;
            }
            else{
                secondGlyphLift.setPower(0);
                secondLiftDirection = DOWN;
            }
        }
        else{

            if(gamepad2.right_stick_y < -LIFT_TWITCH_THRESHOLD){
                secondGlyphLift.setPower(gamepad2.right_stick_y/-1.3);  //76% going up 
                secondLiftDirection = UP;
            }
            else if(gamepad2.right_stick_y > LIFT_TWITCH_THRESHOLD){
                secondGlyphLift.setPower(gamepad2.right_stick_y/-2);  //50% going down
                secondLiftDirection = DOWN;
            }
            else{
                secondGlyphLift.setPower(0);
            }

            linearOpMode.telemetry.addData("Second Lift", "Can move freely");
            linearOpMode.telemetry.update();
        }
    }

    static public CryptoboxDetector initDogeCVForCryptobox(HardwareMap hMap, JDColor color){

        CryptoboxDetector cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hMap.appContext, CameraViewDisplay.getInstance(), FRONT_FACING_CAMERA);

        cryptoboxDetector.downScaleFactor = 0.4;
        if(color == JDColor.RED){
            cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.RED;
        }
        else if(color == JDColor.BLUE){
            cryptoboxDetector.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE;
        }

        cryptoboxDetector.speed = CryptoboxDetector.CryptoboxSpeed.BALANCED;
        cryptoboxDetector.rotateMat = true;



        cryptoboxDetector.enable();

        return cryptoboxDetector;
    }

    static public JewelDetector initDogeCVForJewel(HardwareMap hMap){
        JewelDetector jewelDetector = new JewelDetector();
        jewelDetector.init(hMap.appContext, CameraViewDisplay.getInstance(), FRONT_FACING_CAMERA);
        jewelDetector.downScaleFactor = 0.4;

        jewelDetector.speed = JewelDetector.JewelDetectionSpeed.BALANCED;
        jewelDetector.rotateMat = true;

        jewelDetector.enable();

        return jewelDetector;
    }

    static public void readJewelWithDogeCV(JewelDetector jewelDetector, LinearOpMode linearOpMode){
        while(linearOpMode.opModeIsActive()){
            ElapsedTime mRuntime = new ElapsedTime();

            linearOpMode.telemetry.addData("Status", "Run Time: " + mRuntime.toString());
            Log.d("JDJewel", "Run Time: " + mRuntime.toString());

            linearOpMode.telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
            Log.d("JDJewel", "Current Order: " + jewelDetector.getCurrentOrder().toString());

            linearOpMode.telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result
            Log.d("JDJewel", "Last Known Order: " + jewelDetector.getLastOrder().toString());

            if(linearOpMode.isStopRequested()){
                jewelDetector.disable();
                break;
            }
        }
    }

    static public void lowerJewelArms(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(0.5);
        linearOpMode.sleep(200);
        jewelArm.setPosition(0);
        linearOpMode.sleep(1000);
    }

    static public void raiseJewelArms(LinearOpMode linearOpMode){
        jewelArm.setPosition(0.9);
        linearOpMode.sleep(1000);
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(500);
    }

    static public void knockJewel(JDColor jewelColor, JDColor stoneColor, LinearOpMode linearOpMode){
        if(jewelColor == JDColor.NONE){
            //do nothing
        }
        else if (jewelColor == stoneColor){
            kickOpposite(linearOpMode);
        }
        else if (jewelColor != stoneColor){
            kickSame(linearOpMode);
        }
    }


    static public void kickOpposite(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(0);
        linearOpMode.sleep(1500);
    }

    static public void kickSame(LinearOpMode linearOpMode){
        jewelKnocker.setPosition(1);
        linearOpMode.sleep(1500);
    }


    static public void turn(int degrees, LinearOpMode linearOpMode){
        Orientation angles;

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

        float originalZ = angles.firstAngle;
        float currentZ = angles.firstAngle;

        linearOpMode.telemetry.addData("Current Z", currentZ);
        linearOpMode.telemetry.addData("Original Z", originalZ);
        linearOpMode.telemetry.update();

        if(degrees > 0){
            frontLeftDriveMotor.setPower(0.3);
            frontRightDriveMotor.setPower(0.3);
            backLeftDriveMotor.setPower(0.3);
            backRightDriveMotor.setPower(0.3);

            while((!(currentZ >= degrees - 3) && (currentZ <= degrees + 3)) && linearOpMode.opModeIsActive()){
                angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                currentZ = angles.firstAngle;

                linearOpMode.telemetry.addData("Current Z", currentZ);
                linearOpMode.telemetry.addData("Original Z", originalZ);
                linearOpMode.telemetry.update();
            }
        }
        else{
            frontLeftDriveMotor.setPower(-0.3);
            frontRightDriveMotor.setPower(-0.3);
            backLeftDriveMotor.setPower(-0.3);
            backRightDriveMotor.setPower(-0.3);

            while((!(currentZ <= degrees + 3) && (currentZ >= degrees - 3)) && linearOpMode.opModeIsActive()){
                angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                currentZ = angles.firstAngle;

                linearOpMode.telemetry.addData("Current Z", currentZ);
                linearOpMode.telemetry.addData("Original Z", originalZ);
                linearOpMode.telemetry.update();
            }
        }



        stop();
    }

    static public JDColor detectJewelColor(LinearOpMode linearOpMode){

        float hue = 0;

        JDColor jewelColorFound = JDColor.NONE;

        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        //read color for about 2 seconds
        while( jewelColorFound == JDColor.NONE && mRuntime.time() < 2){

            // hsvValues is an array that will hold the hue, saturation, and value information.
            float hsvValues[] ={0F, 0F, 0F};

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
                    (int) (jewelColorSensor.blue() * SCALE_FACTOR),hsvValues);

            linearOpMode.telemetry.addData("hue", hsvValues[0]);
            linearOpMode.telemetry.addData("S", hsvValues[1]);
            linearOpMode.telemetry.addData("V", hsvValues[2]);
            linearOpMode.telemetry.update();

            hue = hsvValues[0];

            if (hue >= 190 && hue <= 235){
                jewelColorFound = JDColor.BLUE;
            }
            else if (hue <= 15 || hue >= 350){
                jewelColorFound = JDColor.RED;
            }
            else if (hue == 0){
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

    static public RelicRecoveryVuMark getVumark(LinearOpMode linearOpMode, HardwareMap hMap){
        ClosableVuforiaLocalizer vuforia;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = " AZcIMlr/////AAAAGe1W/L9P20hXupxJsIH5bIMDl46JPwjrX2kI+L6+tigIG9bhthzvrEWVBni6g4Jkvs76N/hIT0bFun78pnNDqkG3ZP24XLj45VHA2rYKp8UDww/vfy8xrtvHxedihdX1A2vMWg8Ub8tLjBMgEAqcAYYUMwPRQfI61KQmXvAJBV79XtQughxCh/fbrtoux6WV6HHs8OydP7kPUaUU3f0z5ZOF/TUvcqFFotqnLg/KwXMxxrouRyDGCIbpbP7cYabiR7ShIGvrYoRKtbpwxS3WLSjjTd7ynvoidYipWZ60e6t+wUCzdXahS8g0veYuTQ+vwBqljhtLUWnCUjbJh2jocjxV9kLGgqlPFCmLHZyurYkX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = new ClosableVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //For debug purposes
        //Works here
        //runningOpMode.addTelemetry("VuMark Detection State", "Starting", true);

        relicTrackables.activate();
        //timer based fail safe logic
        long startTime = System.nanoTime();
        long elapsedTime = 0;

        //try to read the vumark until we find a valid vumark or for 3 seconds
        while(vuMark == RelicRecoveryVuMark.UNKNOWN  &&  elapsedTime <= 3000 && linearOpMode.opModeIsActive()){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                linearOpMode.telemetry.addData("Vumark Found", vuMark.toString());
                linearOpMode.telemetry.update();
                break;
            }
            elapsedTime = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime);
            linearOpMode.telemetry.addData("Elapsed Time to find VuMark:", Long.toString(elapsedTime));
            linearOpMode.telemetry.update();
        }

        vuforia.close();

        return vuMark;

    }


    static public void moveUntilCryptoWall(double distanceToWall, RelicRecoveryVuMark vuMark, int color, LinearOpMode linearOpMode){
        int targetColumn;

        if(vuMark == RelicRecoveryVuMark.LEFT && color == RED){
            targetColumn = 1;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER && color == RED){
            targetColumn = 2;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT && color == RED){
            targetColumn = 3;
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT && color == BLUE){
            targetColumn = 1;
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER && color == BLUE){
            targetColumn = 2;
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT && color == BLUE){
            targetColumn = 3;
        }
        else{
            targetColumn = 1;
        }

        int columnsPassed = 0;


        double distanceToCryptoBoxWall = distanceToWall - 5;


        if(color == RED){
            frontLeftDriveMotor.setPower(0.2);
            frontRightDriveMotor.setPower(-0.2);
            backLeftDriveMotor.setPower(0.2);
            backRightDriveMotor.setPower(-0.2);
        }
        else if(color == BLUE){
            frontLeftDriveMotor.setPower(-0.2);
            frontRightDriveMotor.setPower(0.2);
            backLeftDriveMotor.setPower(-0.2);
            backRightDriveMotor.setPower(0.2);
        }

        frontLeftDriveMotor.setPower(0.2);
        frontRightDriveMotor.setPower(-0.2);
        backLeftDriveMotor.setPower(0.2);
        backRightDriveMotor.setPower(-0.2);


        while(linearOpMode.opModeIsActive()){
            if (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall){
                columnsPassed++;

                while (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall && linearOpMode.opModeIsActive()){
                }
            }

            if (columnsPassed >= targetColumn){
                stop();
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
    static public double readAndFilterRangeSensor(LinearOpMode linearOpMode){
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        double distance =  sideRangeSensor.cmUltrasonic();
        while ( (distance == 255 || distance == 0)  && linearOpMode.opModeIsActive() ){
            if(mRuntime.milliseconds() >= 200){
                break;
            }
            distance = sideRangeSensor.cmUltrasonic();
        }
        return distance;
    }

    static int targetColumn(RelicRecoveryVuMark vuMark, JDColor allianceColor, FIELD_SIDE fieldSide)
    {
        int targetColumn = 1;

        switch (vuMark)
        {
            case LEFT:
                if (allianceColor == JDColor.RED) {
                    targetColumn = 3;
                }
                else if (allianceColor == JDColor.BLUE){
                    targetColumn = 1;
                }
                break;

            case CENTER:
                targetColumn = 2;
                break;

            case RIGHT:
                if (allianceColor == JDColor.RED) {
                    targetColumn = 1;
                }
                else if (allianceColor == JDColor.BLUE){
                    targetColumn = 3;
                }
                break;

            default:
                targetColumn = 1;
                break;
        }

        return targetColumn;
    }

    static public void moveUntilCryptoWallv2(double startDistance, RelicRecoveryVuMark vuMark, JDColor allianceColor, FIELD_SIDE fieldSide, LinearOpMode linearOpMode){
        int targetColumn = targetColumn(vuMark, allianceColor, fieldSide);
        int cryptoWallMinVal = 5;

        int columnsPassed = 0;
        boolean firstTime=true;

        double  distanceToCrypto = startDistance - cryptoWallMinVal;

        double motorSpeedRed = 0.20;
        double motorSpeedBlue = 0.20;

        if(allianceColor == JDColor.RED) {
            frontLeftDriveMotor.setPower(motorSpeedRed);
            frontRightDriveMotor.setPower(-motorSpeedRed);
            backLeftDriveMotor.setPower(motorSpeedRed);
            backRightDriveMotor.setPower(-motorSpeedRed);
        }
        else if(allianceColor == JDColor.BLUE){
            frontLeftDriveMotor.setPower(-motorSpeedBlue);
            frontRightDriveMotor.setPower(motorSpeedBlue);
            backLeftDriveMotor.setPower(-motorSpeedBlue);
            backRightDriveMotor.setPower(motorSpeedBlue);
        }

        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        String msg="";

        double distance = readAndFilterRangeSensor(linearOpMode);

        boolean firstTimeIncrementingColumnPassed = true;

        while ( linearOpMode.opModeIsActive() ){

            distance = readAndFilterRangeSensor(linearOpMode);

            // move robot past each crypto column
            //as soon as target column is seen break out of the loop and stop.
            //sometimes the distance is less than the minimum distance of 5, so check if less than 5 or less than 4
            while (  (distance <= distanceToCrypto || distance <= distanceToCrypto-1) && linearOpMode.opModeIsActive()){

                msg = Double.toString(mRuntime.milliseconds()) + ": "
                        + Double.toString(distance)
                        + " Column: " + Integer.toString(columnsPassed)
                        + " CryptoDistance: " + distanceToCrypto;
                linearOpMode.telemetry.addData("range:", msg);
                linearOpMode.telemetry.update();
                Log.d("JDRange", msg);

                //column increased only the first time when there is a change in distance

                if(firstTime && !(targetColumn == 3 && firstTimeIncrementingColumnPassed)){
                    columnsPassed++;
                    firstTime = false;
                    Log.d("JDRange", "Incremented columnsPassed");
                }
                else if(targetColumn == 3 && firstTimeIncrementingColumnPassed){
                    Log.d("JDRange", "Sleep 100 Milliseconds");
                    linearOpMode.sleep(100);
                    firstTimeIncrementingColumnPassed = false;
                }
                else if(firstTime){
                    columnsPassed++;
                    firstTime = false;
                    Log.d("JDRange", "1, 2 Incremented");
                }


                //target column is reached break out the while loop reading range sensor data
                //if last column let it stop at the 4th column, else let it stop at the exact column
                if (columnsPassed >= targetColumn || mRuntime.milliseconds() >= MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS){
                    break;
                }

                distance = readAndFilterRangeSensor(linearOpMode);
            }

            //reset the first time for the next columns
            if (!firstTime) {
                firstTime = true;

            }

            //adjust the minimum distance based on the new reading as the robot might have drifted
            distanceToCrypto = distance - cryptoWallMinVal;



            //target column is reached or if estimate time elapsed, got to break and stop the robot
            //if last column let it stop at the 4th column, else let it stop at the exact column
            if(columnsPassed >= targetColumn || mRuntime.milliseconds() >= MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS ){
                msg = Double.toString(mRuntime.milliseconds()) + ": "
                        + Double.toString(distance)
                        + " Column: " + Integer.toString(columnsPassed)
                        + " CryptoDistance: " + distanceToCrypto;
                linearOpMode.telemetry.addData("range:", msg);
                linearOpMode.telemetry.update();
                Log.d("JDRange", msg);
                if(allianceColor == JDColor.RED) {
                    if (targetColumn == 3) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 1200 MS");
                        moveForTime(-motorSpeedRed, 1200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved backwards for 1200 MS");
                    } else if (targetColumn == 2) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 200 MS");
                        moveForTime(-motorSpeedRed, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved backwards for 200 MS");
                    } else if (targetColumn == 1) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving backwards for 200 MS");
                        moveForTime(-motorSpeedRed, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ":Moved backwards for 200 MS");
                    }

                    break;
                }
                else if(allianceColor == JDColor.BLUE){
                    if (targetColumn == 3) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 1200 MS");
                        moveForTime(motorSpeedBlue, 1200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved forwards for 1200 MS");
                    } else if (targetColumn == 2) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 200 MS");
                        moveForTime(motorSpeedBlue, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moved forwards for 200 MS");
                    } else if (targetColumn == 1) {
                        stop();
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ": Moving forwards for 200 MS");
                        moveForTime(motorSpeedBlue, 200, linearOpMode);
                        Log.d("JDRange", Double.toString(mRuntime.milliseconds()) + ":Moved forwards for 200 MS");
                    }

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

        stop();

    }

    //AVT Algorithm to filter range sensor values and return the sampled distance
    //calculate average value
    //AVT algorithm stands for Antonyan Vardan Transform and its implementation explained below.
    //Collect n samples of data
    //Calculate the standard deviation and average value
    //Drop any data that is greater or less than average ± one standard deviation
    //Calculate average value of remaining data

    static public double filterRangeSensorValues(double[] rangeSensorValues){

        //calculate average
        double sum = 0;
        for(int i=0; i < rangeSensorValues.length ; i++)
            sum = sum + rangeSensorValues[i];
        double average = sum / rangeSensorValues.length;

        //calculate standard deviation
        double sd = 0;
        for(int i = 0; i < rangeSensorValues.length; i++){
            sd = sd + Math.pow((rangeSensorValues[i]-average),2);
        }
        double standardDeviation = Math.sqrt(sd/rangeSensorValues.length);

        //Drop any data that is greater or less than average ± one standard deviation
        int newlength=0;
        double newSum=0;
        for(int i = 0; i < rangeSensorValues.length; i++){
            if ( !(rangeSensorValues[i] > average+standardDeviation || rangeSensorValues[i] < average-standardDeviation ) ){
                newlength++;
                newSum= newSum+rangeSensorValues[i];
            }
        }

        //distance is average value of remaining data
        return newSum/newlength;
    }

}
