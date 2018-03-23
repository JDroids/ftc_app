package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands;

/**
 * Created by dansm on 3/22/2018.
 */

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.resources.PID;
import org.firstinspires.ftc.teamcode.roboutils.templates.Command;

public class AutonomousMovement {
    Command turnPID = new Command() {
        public double targetDegrees;
        public boolean gettingCoefficentsThroughUDP = false;

        PID pidClass = new PID();

        double currentDeg;
        double motorSpeed;

        double allowableError = 0.7;

        Orientation angles;

        @Override
        public void run() {
            angles = opMode.robot.drive.imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

            pidClass.setCoeffecients(0.015, 0.0, 0.002);

            double currentDeg = angles.firstAngle;

            this.loop();
        }

        @Override
        public void loop() {
            if ((targetDegrees <= 180 && targetDegrees >= 175) || (targetDegrees >= -180 && targetDegrees <= -175)) {
                Log.d("JDSanityCheck", "Passed if statement");
                if (targetDegrees <= -175) {
                    Log.d("JDSanityCheck", "this shouldn't happen");
                    targetDegrees = 180 - Math.abs(targetDegrees);
                    targetDegrees += 180;

                }

                while ((!(currentDeg > targetDegrees - allowableError && currentDeg < targetDegrees + allowableError)) && opMode.opModeIsActive()) {
                    //Log.d("JDSanityCheck", "Passed while loop");
                    angles = opMode.robot.drive.imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

                    currentDeg = angles.firstAngle;

                    Log.d("JDSanityCheck", "Current Degrees Before Math: " + currentDeg);

                    if (currentDeg < 0) {
                        currentDeg = 180 - Math.abs(currentDeg);
                        currentDeg += 180;
                    }

                    Log.d("JDSanityCheck", "Current Degrees After Math: " + currentDeg);

                    if (gettingCoefficentsThroughUDP) {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg, true);
                    } else {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg);
                    }

                    opMode.robot.drive.setMotorPower(motorSpeed, motorSpeed, motorSpeed, motorSpeed);

                    angles = opMode.robot.drive.imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

                    opMode.robot.update();
                }
            }
            else {
                while ((!(currentDeg > targetDegrees - allowableError && currentDeg < targetDegrees + allowableError)) && opMode.opModeIsActive()) {
                    angles = opMode.robot.drive.imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

                    currentDeg = angles.firstAngle;

                    if (gettingCoefficentsThroughUDP) {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg, true);
                    } else {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg);
                    }

                    opMode.robot.drive.setMotorPower(motorSpeed, motorSpeed, motorSpeed, motorSpeed);

                    angles = opMode.robot.drive.imuSensor.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

                    opMode.robot.update();
                }

            }
            if (gettingCoefficentsThroughUDP) {
                Log.d("JDPID", "Shutting down Udp receiver");
                pidClass.pidUdpReceiver.shutdown();
            }
            this.stop();
        }

        @Override
        public void stop() {
            opMode.robot.drive.stopDriveMotors();
            opMode.robot.update();
        }
    };
}
