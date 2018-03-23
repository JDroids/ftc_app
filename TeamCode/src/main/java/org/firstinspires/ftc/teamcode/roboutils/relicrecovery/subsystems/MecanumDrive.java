package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import android.graphics.PorterDuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.DriveTrain;

/**
 * Created by dansm on 3/21/2018.
 */

public class MecanumDrive extends DriveTrain {
    public Orientation imuAngularOrientation;
    public double frontRangeSensorDistance;
    public double sideRangeSensorDistance;
    public double rearRangeSensorDistance;

    //These are under MecanumDrive as they are uniquely for movement
    BNO055IMU imuSensor;
    ModernRoboticsI2cRangeSensor frontRangeSensor;
    ModernRoboticsI2cRangeSensor sideRangeSensor;
    ModernRoboticsI2cRangeSensor rearRangeSensor;

    CustomOpMode opMode;

    public void initHardware(CustomOpMode opMode) {
        this.opMode = opMode;

        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontRight"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackRight"));

        imuSensor = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        frontRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        sideRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sideRange");
        rearRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rearRange");

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        this.opMode = opMode;
    }

    public void update(){
        for (DcMotorEx motor : motors) {
            motor.setPower(motorSpeeds.get(motors.indexOf(motor)));
        }

        imuAngularOrientation = imuSensor.getAngularOrientation();

        frontRangeSensorDistance = readAndFilterRangeSensorValues(frontRangeSensor, this.opMode);
        sideRangeSensorDistance = readAndFilterRangeSensorValues(sideRangeSensor, this.opMode);
        rearRangeSensorDistance = readAndFilterRangeSensorValues(rearRangeSensor, this.opMode);
    }

    public void setMotorPower(double frontLeftMotorPower, double frontRightMotorPower, double backLeftMotorPower, double backRightMotorPower) {
        motorSpeeds.set(0, frontLeftMotorPower);
        motorSpeeds.set(1, frontRightMotorPower);
        motorSpeeds.set(2, backLeftMotorPower);
        motorSpeeds.set(3, backRightMotorPower);
    }

    public void stopDriveMotors() {
        moveAtPower(0);
    }

    public void moveAtPower(double power) {
        setMotorPower(-power, power, -power, power);
    }

    static public double readAndFilterRangeSensorValues(ModernRoboticsI2cRangeSensor ultrasonicSensor, LinearOpMode linearOpMode) {
        ElapsedTime mRuntime = new ElapsedTime();
        mRuntime.reset();

        double distance = ultrasonicSensor.cmUltrasonic();
        while ((distance == 255 || distance == 0) && linearOpMode.opModeIsActive()) {
            distance = ultrasonicSensor.cmUltrasonic();
        }
        return distance;
    }

}
