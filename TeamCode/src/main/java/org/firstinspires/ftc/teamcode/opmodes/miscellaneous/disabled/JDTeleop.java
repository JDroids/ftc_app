package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.constants.GRABBERS;
import static org.firstinspires.ftc.teamcode.resources.constants.TELEOP;
import static org.firstinspires.ftc.teamcode.resources.functions.closeGrabber;
import static org.firstinspires.ftc.teamcode.resources.functions.controlGlyphLifts;
import static org.firstinspires.ftc.teamcode.resources.functions.initServos;
import static org.firstinspires.ftc.teamcode.resources.functions.moveArcade;
import static org.firstinspires.ftc.teamcode.resources.functions.openGrabber;
import static org.firstinspires.ftc.teamcode.resources.functions.openGrabberWide;
import static org.firstinspires.ftc.teamcode.resources.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.imuSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicExtender;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicRotationalServo;


/**
 * Created by dansm on 12/7/2017.
 */

@Disabled

@TeleOp(name = "JDTeleOp")

public class JDTeleop extends LinearOpMode {

    @Override

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

        GRABBERS controlledGrabbers = GRABBERS.BOTH_GRABBERS;

        initServos(TELEOP);


        //moveFirstLiftForTime(0.7, 500, this);
        //moveSecondLiftForTime(0.7, 500, this);

        while (opModeIsActive()) {
            moveArcade(gamepad1, this);

            //To change which set of grabbers should be used
            if (gamepad2.dpad_up) {
                controlledGrabbers = GRABBERS.TOP_GRABBER;
            } else if (gamepad2.dpad_down) {
                controlledGrabbers = GRABBERS.BOTTOM_GRABBER;
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                controlledGrabbers = GRABBERS.BOTH_GRABBERS;
            }

            //To open/close grabber
            if (gamepad2.a) {
                closeGrabber(controlledGrabbers);
            } else if (gamepad2.b) {
                openGrabber(controlledGrabbers);
            } else if (gamepad2.y) {
                openGrabberWide(controlledGrabbers);
            }

            //To extend/detract cascading rail
            if (gamepad2.right_bumper) {
                if (gamepad2.x) {
                    relicExtender.setPower(0.5);
                } else {
                    relicExtender.setPower(0.1);
                }
            } else if (gamepad2.left_bumper) {
                if (gamepad2.x) {
                    relicExtender.setPower(-0.5);
                } else {
                    relicExtender.setPower(-0.1);
                }

            } else {
                relicExtender.setPower(0);
            }

            /*if (gamepad1.y) { //To collect relic
                if (relicRotationalServo.getPosition() < 0.775) {
                    relicRotationalServo.setPosition(relicRotationalServo.getPosition() + 0.008);
                } else if (relicRotationalServo.getPosition() > 0.775) {
                    relicRotationalServo.setPosition(relicRotationalServo.getPosition() - 0.008);
                }

                if (relicExtensionServo.getPosition() < 0.3) {
                    relicExtensionServo.setPosition(relicExtensionServo.getPosition() + 0.008);
                } else if (relicExtensionServo.getPosition() > 0.3) {
                    relicExtensionServo.setPosition(relicExtensionServo.getPosition() - 0.008);
                }
            } else {
                //To extend/detract the linear servo on the relic mechanism

                if (gamepad1.right_bumper) { //To open

                    if (relicExtensionServo.getPosition() < 0.9) {
                        relicExtensionServo.setPosition(relicExtensionServo.getPosition() + 0.01);
                    }
                } else if (gamepad1.left_bumper) { //To close

                    if (relicExtensionServo.getPosition() > 0.3) {
                        relicExtensionServo.setPosition(relicExtensionServo.getPosition() - 0.01);
                    }
                }


                //To move the rotational servo on the relic mechanism
                if (gamepad1.a && relicRotationalServo.getPosition() < 0.95) { //Go down, stops at 0.95
                    if (relicRotationalServo.getPosition() < 0.9 && gamepad1.x) { //stops at 0.9 if x PRESSED
                        relicRotationalServo.setPosition(relicRotationalServo.getPosition() + 0.008);
                    } else if (relicRotationalServo.getPosition() < 0.95) { //stops at 0.95 if x NOT pressed
                        relicRotationalServo.setPosition(relicRotationalServo.getPosition() + 0.008);
                    }

                } else if (gamepad1.b && relicRotationalServo.getPosition() > 0.4) { //Go up, stops at 0.4
                    relicRotationalServo.setPosition(relicRotationalServo.getPosition() - 0.008);
                }
            }
            */
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
