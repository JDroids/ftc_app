package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.DriveTrain;

import java.util.ArrayList;

/**
 * Created by dansm on 3/21/2018.
 */

public class MecanumDrive extends DriveTrain {
    public Orientation imuAngularOrientation;
    public double frontRangeSensorDistance;
    public double sideRangeSensorDistance;
    public double rearRangeSensorDistance;

    public int frontLeftEncoderTicks;
    public int frontRightEncoderTicks;
    public int backLeftEncoderTicks;
    public int backRightEncoderTicks;

    public double heading;
    public Waypoint position;

    public DcMotorEx.RunMode motorRunMode = DcMotorEx.RunMode.RUN_USING_ENCODER;

    public boolean areMotorsBusy = false;

    int leftSideTicks;
    int rightSideTicks;

    double leftSideDistance;
    double rightSideDistance;

    double distance;
    double previousDistance;

    double changeInDistance;

    public double averageEncoderTicks;

    double changeInX;

    //These are under MecanumDrive as they are uniquely for movement
    BNO055IMU imuSensor;
    ModernRoboticsI2cRangeSensor frontRangeSensor;
    ModernRoboticsI2cRangeSensor sideRangeSensor;
    ModernRoboticsI2cRangeSensor rearRangeSensor;

    CustomOpMode opMode;

    public void initHardware(CustomOpMode opMode) {
        this.opMode = opMode;

        this.motors = new ArrayList<DcMotorEx>();
        this.motorSpeeds = new ArrayList<Double>();

        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontRight"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackRight"));

        imuSensor = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor.initialize(parameters);

        frontRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        sideRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sideRange");
        rearRangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rearRange");

        motorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

        this.opMode = opMode;
    }

    public int convertFromCMToTicks(double cm) {
        return (int) (cm/10 * Math.PI * (1120/16));
        /*
        1120 = Bare NeveRest ticks per revolution
        16 = our motor reduction of 1:16
        10 = the radius of our wheels in cm
        */
    }

    public double convertFromTicksToCM(int ticks) {
        return (ticks / (1120/16)) * (10 * Math.PI);
        /*
        1120 = Bare NeveRest ticks per revolution
        16 = our motor reduction of 16:1
        10 = the radius of our wheels in cm
        */
    }

    public void setToRelativePosition(int frontLeftTicks, int frontRightTicks, int backLeftTicks, int backRightTicks){
        for(DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motors.get(0).setTargetPosition(frontLeftEncoderTicks + frontLeftTicks);
        motors.get(1).setTargetPosition(frontRightEncoderTicks + frontRightTicks);
        motors.get(2).setTargetPosition(backLeftEncoderTicks + backLeftTicks);
        motors.get(3).setTargetPosition(backRightEncoderTicks + backRightTicks);
    }

    public void setAllMotorsToRelativePosition(int position) {
        setToRelativePosition(-position, position, -position, position);
    }

    int timesRun = 0;

    public void update() {
        if (timesRun == 0) {
            motorRunMode = DcMotorEx.RunMode.STOP_AND_RESET_ENCODER;
        }
        else if(timesRun == 1){
            motorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
        }

        for (DcMotorEx motor : motors) {
            motor.setMode(motorRunMode);
            motor.setPower(motorSpeeds.get(motors.indexOf(motor)));
        }

        areMotorsBusy = motors.get(0).isBusy() || motors.get(1).isBusy() || motors.get(2).isBusy() || motors.get(3).isBusy();

        previousDistance = distance;

        frontLeftEncoderTicks = motors.get(0).getCurrentPosition();
        frontRightEncoderTicks = motors.get(1).getCurrentPosition();
        backLeftEncoderTicks = motors.get(2).getCurrentPosition();
        backRightEncoderTicks = motors.get(3).getCurrentPosition();

        leftSideTicks = (-frontLeftEncoderTicks + -backLeftEncoderTicks) / 2;
        rightSideTicks = (frontRightEncoderTicks + backRightEncoderTicks) / 2;

        leftSideDistance = convertFromTicksToCM(leftSideTicks);
        rightSideDistance = convertFromTicksToCM(rightSideTicks);

        distance = (leftSideDistance + rightSideDistance) / 2;

        changeInDistance = distance - previousDistance;

        imuAngularOrientation = imuSensor.getAngularOrientation();

        heading = ((imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES).firstAngle + 180) * -1) * (Math.PI / 180);
        //In radians

        position.heading = heading;
        
        position.x = position.x + changeInDistance * Math.cos(position.heading);
        position.y = position.y + changeInDistance * Math.sin(position.heading);


        frontRangeSensorDistance = readAndFilterRangeSensorValues(frontRangeSensor, this.opMode);
        sideRangeSensorDistance = readAndFilterRangeSensorValues(sideRangeSensor, this.opMode);
        rearRangeSensorDistance = readAndFilterRangeSensorValues(rearRangeSensor, this.opMode);


        timesRun++;
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
