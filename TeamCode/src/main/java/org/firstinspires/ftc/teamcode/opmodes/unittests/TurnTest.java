package org.firstinspires.ftc.teamcode.opmodes.unittests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.functions.turnPID;
import static org.firstinspires.ftc.teamcode.resources.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;

/**
 * Created by dansm on 3/10/2018.
 */

@Disabled
@Autonomous(name = "TurnTest")

public class TurnTest extends LinearOpMode {
    public void runOpMode() {
        initHardwareMap(hardwareMap);

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor.initialize(parameters);

        turnPID(90, this);

        sleep(100);

        turnPID(180, this);

    }
}
