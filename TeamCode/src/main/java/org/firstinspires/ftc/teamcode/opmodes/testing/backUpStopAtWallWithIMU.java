package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by dansm on 2/11/2018.
 */

@Autonomous(name="Back Up To Wall")

public class backUpStopAtWallWithIMU extends LinearOpMode{
    @Override
    public void runOpMode(){
        BNO055IMU imuSensor = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuSensor.initialize(parameters);

        ElapsedTime mRuntime = new ElapsedTime();

        waitForStart();

        Acceleration currentAccel;

        double currentZAccel;

        mRuntime.reset();
        while(opModeIsActive()){
            currentAccel = imuSensor.getAcceleration();


            currentZAccel = currentAccel.zAccel;

            telemetry.addData("Acceleration", currentZAccel);
            telemetry.addData("MS Elapsed", mRuntime.milliseconds());
            Log.d("Acceleration", Double.toString(mRuntime.milliseconds()) + " " + Double.toString(currentZAccel));

            telemetry.update();
        }

    }
}
