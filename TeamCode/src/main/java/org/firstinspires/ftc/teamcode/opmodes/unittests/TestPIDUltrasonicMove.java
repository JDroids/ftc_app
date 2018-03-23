package org.firstinspires.ftc.teamcode.opmodes.unittests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.teamcode.resources.functions.moveToDistanceUltrasonicPID;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRangeSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;


/**
 * Created by dansm on 3/3/2018.
 */
@Disabled

@Autonomous(name = "HowToGiveNarmadaAHeartAttack")

public class TestPIDUltrasonicMove extends LinearOpMode {
    public void runOpMode() {
        initHardwareMap(hardwareMap);

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuSensor.initialize(parameters);

        imuSensor.startAccelerationIntegration(new Position(), new Velocity(), 250);

        moveToDistanceUltrasonicPID(frontRangeSensor, 30, this);

        imuSensor.stopAccelerationIntegration();
    }
}
