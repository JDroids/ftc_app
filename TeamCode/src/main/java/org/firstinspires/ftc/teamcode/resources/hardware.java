
package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import java.lang.reflect.Field;

/**
 * Created by dansm on 12/13/2017.
 */

public class hardware{
    static public DcMotor frontLeftDriveMotor = null;
    static public DcMotor frontRightDriveMotor = null;
    static public DcMotor backLeftDriveMotor = null;
    static public DcMotor backRightDriveMotor = null;

    static public DcMotor firstGlyphLift = null;
    static public DcMotor secondGlyphLift = null;

    static public DcMotor relicExtender = null;
    static public Servo relicRotationalServo;
    static public Servo relicLinearServo;

    static public Servo glyphGrabberTL = null;
    static public Servo glyphGrabberTR = null;
    static public Servo glyphGrabberBL = null;
    static public Servo glyphGrabberBR = null;

    static public Servo jewelKnocker = null;
    static public Servo jewelArm = null;

    static public DigitalChannel firstLiftSwitch = null;
    static public DigitalChannel secondLiftSwitch = null;

    static public ColorSensor jewelColorSensor = null;
    static public DistanceSensor jewelDistanceSensor = null;

    static public ModernRoboticsI2cRangeSensor rearRangeSensor = null;

    static public BNO055IMU imuSensor = null;

    static public void initHardwareMap(HardwareMap map){
        HardwareMap hMap = map;


        frontLeftDriveMotor = hMap.dcMotor.get("FrontLeft");
        frontRightDriveMotor = hMap.dcMotor.get("FrontRight");
        backLeftDriveMotor = hMap.dcMotor.get("BackLeft");
        backRightDriveMotor = hMap.dcMotor.get("BackRight");

        firstGlyphLift = hMap.dcMotor.get("MotorGlyphLift");
        secondGlyphLift = hMap.dcMotor.get("MotorGlyphLift2");

        relicExtender = hMap.dcMotor.get("relicMotor");
        relicRotationalServo = hMap.servo.get("relicPivot");
        relicLinearServo = hMap.servo.get("relicLinear");

        glyphGrabberTL = hMap.servo.get("glyphGrabberTL");
        glyphGrabberTR = hMap.servo.get("glyphGrabberTR");
        glyphGrabberBL = hMap.servo.get("glyphGrabberBL");
        glyphGrabberBR = hMap.servo.get("glyphGrabberBR");

        jewelKnocker = hMap.servo.get("servoJewelKnock");
        jewelArm = hMap.servo.get("servoJewelArm");

        firstLiftSwitch = hMap.digitalChannel.get("FirstLiftSwitch");
        secondLiftSwitch = hMap.digitalChannel.get("SecondLiftSwitch");

        firstLiftSwitch.setMode(DigitalChannel.Mode.INPUT);
        secondLiftSwitch.setMode(DigitalChannel.Mode.INPUT);

        imuSensor = hMap.get(BNO055IMU.class, "imu");

        jewelColorSensor = hMap.colorSensor.get("color1");

        jewelDistanceSensor = hMap.get(DistanceSensor.class, "color1");

        rearRangeSensor = hMap.get(ModernRoboticsI2cRangeSensor.class, "rearRange");

    }
}