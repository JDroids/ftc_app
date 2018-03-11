package org.firstinspires.ftc.teamcode.opmodes.competition.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.resources.constants;

import static org.firstinspires.ftc.teamcode.resources.constants.TELEOP;
import static org.firstinspires.ftc.teamcode.resources.functions.closeGrabber;
import static org.firstinspires.ftc.teamcode.resources.functions.controlGlyphLifts;
import static org.firstinspires.ftc.teamcode.resources.functions.initServos;
import static org.firstinspires.ftc.teamcode.resources.functions.moveArcadeFOD;
import static org.firstinspires.ftc.teamcode.resources.functions.openGrabber;
import static org.firstinspires.ftc.teamcode.resources.functions.openGrabberWide;
import static org.firstinspires.ftc.teamcode.resources.functions.relicControl;
import static org.firstinspires.ftc.teamcode.resources.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicExtender;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicExtensionServo;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicRotationalServo;

/**
 * Created by dansm on 3/2/2018.
 */

@TeleOp(name = "REDJudgeTeleop")

public class redJudgeTeleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        //Code to run after init is pressed

        initHardwareMap(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imuSensor.initialize(parameters);

        waitForStart();
        //Code to run after play is pressed

        constants.GRABBERS controlledGrabbers = constants.GRABBERS.BOTH_GRABBERS;

        initServos(TELEOP);


        //moveFirstLiftForTime(0.7, 500, this);
        //moveSecondLiftForTime(0.7, 500, this);

        while (opModeIsActive()) {
            moveArcadeFOD(gamepad1, constants.FIELD_SIDE.JUDGE_SIDE, constants.JDColor.RED, this);

            //To change which set of grabbers should be used
            if (gamepad2.dpad_up) {
                controlledGrabbers = constants.GRABBERS.TOP_GRABBER;
            } else if (gamepad2.dpad_down) {
                controlledGrabbers = constants.GRABBERS.BOTTOM_GRABBER;
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                controlledGrabbers = constants.GRABBERS.BOTH_GRABBERS;
            }

            //To open/close grabber
            if (gamepad2.a) {
                closeGrabber(controlledGrabbers);
            } else if (gamepad2.b) {
                openGrabber(controlledGrabbers);
            } else if (gamepad2.y) {
                openGrabberWide(controlledGrabbers);
            }

            relicControl(gamepad1, gamepad2);

            controlGlyphLifts(gamepad2, this);


            //A bunch of telemetry for nerds
            telemetry.addData("relicRotationalServo", relicRotationalServo.getPosition());
            telemetry.addData("Front Left Motor Power", frontLeftDriveMotor.getPower());
            telemetry.addData("Front Right Motor Power", frontRightDriveMotor.getPower());
            telemetry.addData("Back Left Motor Power", backLeftDriveMotor.getPower());
            telemetry.addData("Back Right Motor Power", backRightDriveMotor.getPower());

            telemetry.update();
        }
    }
}