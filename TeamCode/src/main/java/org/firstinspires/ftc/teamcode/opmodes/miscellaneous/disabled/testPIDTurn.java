package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.hardware.*;
import static org.firstinspires.ftc.teamcode.resources.functions.*;

/**
 * Created by dansm on 2/28/2018.
 */

@Disabled
@Autonomous(name="Test Pid Turn")

public class testPIDTurn extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        initHardwareMap(hardwareMap);

        initServos(false);

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imuSensor.initialize(parameters);

        turnPID(90);
    }
}
