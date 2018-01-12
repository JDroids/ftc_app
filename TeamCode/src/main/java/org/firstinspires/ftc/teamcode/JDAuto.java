package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by kevin on 12/14/17.
 */

@Autonomous(name="JDAutoJava")

public class JDAuto extends LinearOpMode{

    //Add IMU Parameters

    double JEWEL_KNOCKER_INIT_POSITION = 0;
    double JEWEL_ARM_INIT_POSITION = 0.9;

    DcMotor frontLeftDriveMotor = null;
    DcMotor frontRightDriveMotor = null;
    DcMotor backLeftDriveMotor = null;
    DcMotor backRightDriveMotor = null;

    DcMotor firstGlyphLift = null;

    Servo servoGrabberLeft = null;
    Servo servoGrabberRight = null;

    Servo jewelKnocker = null;
    Servo jewelArm = null;

    ColorSensor JewelColor = null;

    @Override

    public void runOpMode() throws InterruptedException {
        frontLeftDriveMotor = hardwareMap.dcMotor.get("FrontLeft");
        frontRightDriveMotor = hardwareMap.dcMotor.get("FrontRight");
        backLeftDriveMotor = hardwareMap.dcMotor.get("BackLeft");
        backRightDriveMotor = hardwareMap.dcMotor.get("BackRight");

        firstGlyphLift = hardwareMap.dcMotor.get("MotorGlyphLift");

        servoGrabberLeft = hardwareMap.servo.get("servoGrabberRight");
        servoGrabberRight = hardwareMap.servo.get("servoGrabberLeft");

        jewelKnocker = hardwareMap.servo.get("servoJewelKnock");
        jewelArm = hardwareMap.servo.get("servoJewelArm");

        JewelColor = hardwareMap.colorSensor.get("color1");

        JEWEL_KNOCKER_INIT_POSITION = 0.5;
        Thread.sleep(200);
        JEWEL_ARM_INIT_POSITION = 0.05;
        Thread.sleep(200);

        int jewelColorFound = -100;
        int stoneFound = -100; // these two lines are for compiling purposes only

        while(opModeIsActive()) {

            waitForStart();



            if(jewelColorFound == 0){}
            //do nothing
            if(jewelColorFound == stoneFound){}
            //knock opposite
            if(jewelColorFound != stoneFound){}
            //knock same


        }
    }
}