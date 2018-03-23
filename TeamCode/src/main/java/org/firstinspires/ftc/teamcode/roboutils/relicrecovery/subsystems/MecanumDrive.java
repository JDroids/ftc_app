package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboutils.templates.DriveTrain;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

import java.util.ArrayList;

/**
 * Created by dansm on 3/21/2018.
 */

public class MecanumDrive extends DriveTrain {
    public BNO055IMU imuSensor;

    public void initHardware(OpMode opMode){
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontRight"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackRight"));

        imuSensor = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        for(DcMotorEx motor : motors){
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setMotorPower(double frontLeftMotorPower, double frontRightMotorPower, double backLeftMotorPower, double backRightMotorPower){
        motorSpeeds.set(0, frontLeftMotorPower);
        motorSpeeds.set(1, frontRightMotorPower);
        motorSpeeds.set(2, backLeftMotorPower);
        motorSpeeds.set(3, backRightMotorPower);
    }

    public void stopDriveMotors(){
        setMotorPower(0, 0, 0, 0);
    }

}
