package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.hardware.backLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.backRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontLeftDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRightDriveMotor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.sideRangeSensor;

/**
 * Created by dansm on 12/15/2017.
 */
@Disabled
@TeleOp(name = "Count Passes")

public class countPasses extends LinearOpMode {
    @Override
    public void runOpMode() {
        int columnsPassed = 0;

        initHardwareMap(hardwareMap);

        waitForStart();

        double distanceToWall = sideRangeSensor.cmUltrasonic();
        sleep(100);
        double distanceToCryptoBoxWall = distanceToWall - 8;

        frontLeftDriveMotor.setPower(0.2);
        frontRightDriveMotor.setPower(-0.2);
        backLeftDriveMotor.setPower(0.2);
        backRightDriveMotor.setPower(-0.2);
        while (opModeIsActive()) {
            if (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall) {
                columnsPassed++;

                while (sideRangeSensor.cmUltrasonic() <= distanceToCryptoBoxWall && opModeIsActive()) {
                }
            }
            if (columnsPassed >= 2) {
                frontLeftDriveMotor.setPower(0);
                frontRightDriveMotor.setPower(0);
                backLeftDriveMotor.setPower(0);
                backRightDriveMotor.setPower(0);

                //turn(90, this);
            }

            telemetry.addData("Distance to Wall", distanceToWall);
            telemetry.addData("Distance to Crypto Box Wall", distanceToCryptoBoxWall);
            telemetry.addData("Centimeters from Object", sideRangeSensor.cmUltrasonic());
            telemetry.addData("Columns Passed", columnsPassed);
            telemetry.update();
        }

    }
}
