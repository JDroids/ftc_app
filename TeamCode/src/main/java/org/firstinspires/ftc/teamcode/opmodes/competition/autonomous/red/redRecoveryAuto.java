package org.firstinspires.ftc.teamcode.opmodes.competition.autonomous.red;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.resources.external.ClosableVuforiaLocalizer;

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
import static org.firstinspires.ftc.teamcode.resources.functions.initVuforia;
import static org.firstinspires.ftc.teamcode.resources.functions.moveFirstLiftForTime;
import static org.firstinspires.ftc.teamcode.resources.functions.moveForTime;
import static org.firstinspires.ftc.teamcode.resources.functions.moveToCryptoColumnEncoders;
import static org.firstinspires.ftc.teamcode.resources.functions.moveUntilCryptoWallUsingUltrasonicv2;
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

@Autonomous(name = "REDRecoveryAuto")
public class redRecoveryAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Code to run after init is pressed

        initHardwareMap(hardwareMap);

        initServos(AUTONOMOUS);

        ClosableVuforiaLocalizer vuforia = initVuforia(hardwareMap);

        double distanceToWall = readAndFilterRangeSensorValues(sideRangeSensor, this);

        while (!isStarted()) {
            distanceToWall = readAndFilterRangeSensorValues(sideRangeSensor, this);
            telemetry.addData("Distance to wall", distanceToWall);
            telemetry.addData("Rear Range: ", readAndFilterRangeSensorValues(rearRangeSensor, this));
            telemetry.addData("Front Range: ", readAndFilterRangeSensorValues(frontRangeSensor, this));
            telemetry.update();
        }

        waitForStart();
        //Code to run after play is pressed

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor.initialize(parameters);

        //detect the VuMark
        telemetry.addData("Vumark:", "Initializing");
        telemetry.update();
        RelicRecoveryVuMark vuMark = getVumark(vuforia, this);
        telemetry.addData("Vumark:", vuMark.toString());
        telemetry.update();

        doAllJewelStuff(JDColor.RED, this);

        sleep(300);

        //grab the block
        closeGrabber(BOTH_GRABBERS);

        sleep(500);

        moveFirstLiftForTime(GLYPH_LIFT_AUTO_SPEED, 1500, this);

        sleep(100);

        moveForTime(-0.2, 250, this);

        sleep(400);

        ElapsedTime globalRuntime = new ElapsedTime();
        globalRuntime.reset();

        //go to cryptobox
        moveUntilCryptoWallUsingUltrasonicv2(distanceToWall, RelicRecoveryVuMark.RIGHT, JDColor.RED, FIELD_SIDE.RECOVERY_SIDE, this, globalRuntime);
        //^ "This force goes to the first wall, this is a complete hack sry <3" -Daniel

        sleep(400);

        globalRuntime.reset();

        moveToCryptoColumnEncoders(vuMark, JDColor.RED, FIELD_SIDE.RECOVERY_SIDE, this);

        sleep(100);

        globalRuntime.reset();

        turnPID(90);

        sleep(100);

        depositGlyph(this);


        //time to look for the second and third glyph

    }
}
