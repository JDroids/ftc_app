package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dansm on 12/13/2017.
 */

public class hardware {
    static public DcMotorEx frontLeftDriveMotor = null;
    static public DcMotorEx frontRightDriveMotor = null;
    static public DcMotorEx backLeftDriveMotor = null;
    static public DcMotorEx backRightDriveMotor = null;

    static public DcMotor firstGlyphLift = null;
    static public DcMotor secondGlyphLift = null;

    static public DcMotor relicExtender = null;
    static public Servo relicRotationalServo;
    static public Servo relicExtensionServo;

    static public Servo glyphGrabberTL = null;
    static public Servo glyphGrabberTR = null;
    static public Servo glyphGrabberBL = null;
    static public Servo glyphGrabberBR = null;

    static public Servo jewelKnocker = null;
    static public Servo jewelArm = null;

    static public DigitalChannel firstLiftTopSwitch = null;
    static public DigitalChannel firstLiftBottomSwitch = null;
    static public DigitalChannel secondLiftTopSwitch = null;
    static public DigitalChannel secondLiftBottomSwitch = null;

    static public ColorSensor jewelColorSensor = null;
    static public DistanceSensor jewelDistanceSensor = null;

    static public ModernRoboticsI2cRangeSensor sideRangeSensor = null;

    static public ModernRoboticsI2cRangeSensor frontRangeSensor = null;

    static public ModernRoboticsI2cRangeSensor rearRangeSensor = null;


    static public BNO055IMU imuSensor = null;

    static public void initHardwareMap(HardwareMap map) {
        HardwareMap hMap = map;


        frontLeftDriveMotor = (DcMotorEx)hMap.dcMotor.get("FrontLeft");
        frontRightDriveMotor = (DcMotorEx)hMap.dcMotor.get("FrontRight");
        backLeftDriveMotor = (DcMotorEx)hMap.dcMotor.get("BackLeft");
        backRightDriveMotor = (DcMotorEx)hMap.dcMotor.get("BackRight");

        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        firstGlyphLift = hMap.dcMotor.get("MotorGlyphLift");
        secondGlyphLift = hMap.dcMotor.get("MotorGlyphLift2");

        relicExtender = hMap.dcMotor.get("relicMotor");

        relicExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        relicRotationalServo = hMap.servo.get("relicPivot");
        relicExtensionServo = hMap.servo.get("relicLinear");

        glyphGrabberTL = hMap.servo.get("glyphGrabberTL");
        glyphGrabberTR = hMap.servo.get("glyphGrabberTR");
        glyphGrabberBL = hMap.servo.get("glyphGrabberBL");
        glyphGrabberBR = hMap.servo.get("glyphGrabberBR");

        jewelKnocker = hMap.servo.get("servoJewelKnock");
        jewelArm = hMap.servo.get("servoJewelArm");

        firstLiftTopSwitch = hMap.digitalChannel.get("FirstLiftSwitchUpper");
        firstLiftBottomSwitch = hMap.digitalChannel.get("FirstLiftSwitchLower");

        secondLiftTopSwitch = hMap.digitalChannel.get("SecondLiftSwitchUpper");
        secondLiftBottomSwitch = hMap.digitalChannel.get("SecondLiftSwitchLower");

        firstLiftTopSwitch.setMode(DigitalChannel.Mode.INPUT);
        firstLiftBottomSwitch.setMode(DigitalChannel.Mode.INPUT);

        secondLiftTopSwitch.setMode(DigitalChannel.Mode.INPUT);
        secondLiftBottomSwitch.setMode(DigitalChannel.Mode.INPUT);

        imuSensor = hMap.get(BNO055IMU.class, "imu");

        jewelColorSensor = hMap.colorSensor.get("color1");

        jewelDistanceSensor = hMap.get(DistanceSensor.class, "color1");

        sideRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "sideRange");
        frontRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        rearRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "rearRange");

    }
}