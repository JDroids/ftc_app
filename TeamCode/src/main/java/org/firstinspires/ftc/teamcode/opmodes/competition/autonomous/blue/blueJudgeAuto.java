package org.firstinspires.ftc.teamcode.opmodes.competition.autonomous.blue;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.resources.constants.AUTONOMOUS;
import static org.firstinspires.ftc.teamcode.resources.constants.FIELD_SIDE;
import static org.firstinspires.ftc.teamcode.resources.constants.GLYPH_LIFT_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.resources.constants.GRABBERS.BOTH_GRABBERS;
import static org.firstinspires.ftc.teamcode.resources.constants.JDColor;
import static org.firstinspires.ftc.teamcode.resources.functions.closeGrabber;
import static org.firstinspires.ftc.teamcode.resources.functions.depositGlyph;
import static org.firstinspires.ftc.teamcode.resources.functions.doAllJewelStuff;
import static org.firstinspires.ftc.teamcode.resources.functions.getVumark;
import static org.firstinspires.ftc.teamcode.resources.functions.initServos;
import static org.firstinspires.ftc.teamcode.resources.functions.moveFirstLiftForTime;
import static org.firstinspires.ftc.teamcode.resources.functions.moveForTime;
import static org.firstinspires.ftc.teamcode.resources.functions.moveToCryptoColumnEncoders;
import static org.firstinspires.ftc.teamcode.resources.functions.moveToDistanceUltrasonicPID;
import static org.firstinspires.ftc.teamcode.resources.functions.readAndFilterRangeSensorValues;
import static org.firstinspires.ftc.teamcode.resources.functions.turnPID;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRangeSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.rearRangeSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.sideRangeSensor;

/**
 * Created by dansm on 12/21/2017.
 */

@Autonomous(name = "BLUEJudgeAuto")
public class blueJudgeAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Code to run after init is pressed

        initHardwareMap(hardwareMap);

        initServos(AUTONOMOUS);

        double distanceToWall = readAndFilterRangeSensorValues(sideRangeSensor, this);

        while (!isStarted()) {
            distanceToWall = readAndFilterRangeSensorValues(sideRangeSensor, this);
            telemetry.addData("Distance to wall", distanceToWall);
            telemetry.addData("Rear Range: ", readAndFilterRangeSensorValues(rearRangeSensor, this));
            telemetry.addData("Front Range: ", readAndFilterRangeSensorValues(frontRangeSensor, this));
            telemetry.update();
        }

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor.initialize(parameters);

        Orientation angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        Log.d("JDZValue", Double.toString(angles.secondAngle));

        //Code to run after play is pressed

        //detect the VuMark
        telemetry.addData("Vumark:", "Initializing");
        telemetry.update();
        RelicRecoveryVuMark vuMark = getVumark(this, hardwareMap);
        telemetry.addData("Vumark:", vuMark.toString());
        telemetry.update();

        doAllJewelStuff(JDColor.BLUE, this);

        sleep(300);

        //grab the block
        closeGrabber(BOTH_GRABBERS);

        sleep(500);

        moveFirstLiftForTime(GLYPH_LIFT_AUTO_SPEED, 1500, this);

        sleep(100);

        ElapsedTime globalRuntime = new ElapsedTime();
        globalRuntime.reset();

        moveForTime(0.25, 1200, this);

        angles = imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
        Log.d("JDZValue", Double.toString(angles.secondAngle));

        sleep(200);

        globalRuntime.reset();

        moveToDistanceUltrasonicPID(frontRangeSensor, 40, this);

        sleep(100);

        turnPID(90);

        sleep(100);

        globalRuntime.reset();

        moveToDistanceUltrasonicPID(frontRangeSensor, 54, this);

        sleep(100);

        //align with right crypto column
        moveToCryptoColumnEncoders(vuMark, JDColor.BLUE, FIELD_SIDE.JUDGE_SIDE, this);

        sleep(100);

        globalRuntime.reset();

        turnPID(0);

        sleep(100);

        depositGlyph(this);

        sleep(100);

        //time to look for the second and third glyph


    }
}
