package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands;

/**
 * Created by dansm on 3/22/2018.
 */

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.PID;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.templates.Command;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

import java.util.ArrayList;

public class AutonomousMovement {
    public enum RANGE_SENSORS{
        FRONT_RANGE_SENSOR,
        SIDE_RANGE_SENSOR,
        REAR_RANGE_SENSOR
    }

    static public int convertFromCMToTicks(double cm) {
        return (int) ((((((Math.PI * 4) / 16) * 112) * cm) / 4) / 2.54); //Convert the amount of centimeters to ticks
    }

    public Command turn = new Command() {
        double targetDegrees;
        boolean gettingCoefficentsThroughUDP = false;

        double currentDeg;
        double motorSpeed;

        Orientation angles;
        PID pidClass;

        double allowableError = 0.7;

        @Override
        public void run(CustomOpMode opMode, Object... arguments) {
            switch (arguments.length) {
                case 0:
                    //If there are no arguments, throw exception
                    throw new IllegalArgumentException("Not Enough Arguments Given");
                case 1:
                    try {
                        this.targetDegrees = (double) arguments[0];
                    } catch (ClassCastException e) {
                        throw new IllegalArgumentException("Argument(s) of the wrong type (Casting failed)");
                    }

                    break;
                case 2:
                    try {
                        this.targetDegrees = (double) arguments[0];
                        this.gettingCoefficentsThroughUDP = (boolean) arguments[1];
                    } catch (ClassCastException e) {
                        throw new IllegalArgumentException("Argument(s) of the wrong type (Casting failed)");
                    }

                    break;
                default:
                    //If extra arguments are given, throw exception
                    throw new IllegalArgumentException("Too Many Arguments Given");
            }

            this.opMode = opMode;

            opMode.robot.update();
            angles = opMode.robot.drive.imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

            pidClass = new PID();

            pidClass.setCoeffecients(0.007, 0.0, 0.002);

            opMode.robot.update();

            this.loop();
        }

        @Override
        public void loop() {
            currentDeg = angles.firstAngle;
            if ((targetDegrees <= 180 && targetDegrees >= 175) || (targetDegrees >= -180 && targetDegrees <= -175)) {
                Log.d("JDSanityCheck", "Passed if statement");
                if (targetDegrees <= -175) {
                    Log.d("JDSanityCheck", "this shouldn't happen");
                    targetDegrees = 180 - Math.abs(targetDegrees);
                    targetDegrees += 180;

                }

                while ((!(currentDeg > targetDegrees - allowableError && currentDeg < targetDegrees + allowableError)) && opMode.opModeIsActive()) {
                    //Log.d("JDSanityCheck", "Passed while loop");
                    angles = opMode.robot.drive.imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

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

                    opMode.robot.update();

                    angles = opMode.robot.drive.imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                }
            }
            else {
                while ((!(currentDeg > targetDegrees - allowableError && currentDeg < targetDegrees + allowableError)) && opMode.opModeIsActive()) {
                    angles = opMode.robot.drive.imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);

                    currentDeg = angles.firstAngle;

                    if (gettingCoefficentsThroughUDP) {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg, true);
                    } else {
                        motorSpeed = pidClass.calculateOutput(targetDegrees, currentDeg);
                    }

                    Log.d("JDTurn", "Motor Power: "+ motorSpeed);

                    opMode.robot.drive.setMotorPower(motorSpeed, motorSpeed, motorSpeed, motorSpeed);

                    opMode.robot.update();

                    angles = opMode.robot.drive.imuAngularOrientation.toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).toAngleUnit(AngleUnit.DEGREES);
                }

            }

            this.stop();
        }

        @Override
        public void stop(){
            opMode.robot.drive.stopDriveMotors();
            opMode.robot.update();
            if (gettingCoefficentsThroughUDP) {
                Log.d("JDPID", "Shutting down Udp receiver");
                pidClass.pidUdpReceiver.shutdown();
            }
        }
    };

    public Command moveToDistanceUltrasonic = new Command() {
        RANGE_SENSORS rangeSensor;
        int centimeters;
        boolean gettingCoefficentsThroughUDP;
        double motorSpeed;
        ArrayList<Double> distanceOverTime = new ArrayList<Double>();
        double distance;

        PID pidClass;

        double allowableError = 2;

        @Override
        public void run(CustomOpMode opMode, Object... inputs) {
            switch (inputs.length){
                case 0:
                    throw new IllegalArgumentException("Not Enough Arguments Given");
                case 1:
                    throw new IllegalArgumentException("Not Enough Arguments Given");
                case 2:
                    try {
                        rangeSensor = (RANGE_SENSORS) inputs[0];
                        centimeters = (int) inputs[1];
                    }
                    catch (ClassCastException e){
                        throw new IllegalArgumentException("Argument(s) of the wrong type (Casting failed)");
                    }
                    break;
                case 3:
                    try {
                        rangeSensor = (RANGE_SENSORS) inputs[0];
                        centimeters = (int) inputs[1];
                        gettingCoefficentsThroughUDP = (boolean) inputs[2];
                    }
                    catch (ClassCastException e){
                        throw new IllegalArgumentException("Argument(s) of the wrong type (Casting failed)");
                    }
                default:
                    throw new IllegalArgumentException("Too Many Arguments Given");
            }

            pidClass = new PID();

            pidClass.setCoeffecients(0.01, 0, 0.002);

            this.opMode = opMode;

            opMode.robot.update();

            this.loop();
        }

        @Override
        public void loop() {
            while (!(distance > centimeters - allowableError && distance < centimeters + allowableError) && opMode.opModeIsActive()) {
                if(rangeSensor == RANGE_SENSORS.FRONT_RANGE_SENSOR){
                    distance = opMode.robot.drive.frontRangeSensorDistance;
                }
                else if(rangeSensor == RANGE_SENSORS.REAR_RANGE_SENSOR){
                    distance = opMode.robot.drive.rearRangeSensorDistance;
                }

                distanceOverTime.add(distance);

                boolean stopLoop = false;

                if (distanceOverTime.size() >= 15) {
                    stopLoop = true;
                    for (int i = distanceOverTime.size() - 1; i >= 1; i--) {
                        if (!((distanceOverTime.get(i) > (distanceOverTime.get(i - 1) - 1)) && (distanceOverTime.get(i) < (distanceOverTime.get(i - 1) + 1)))) {
                            stopLoop = false;
                        }
                    }

                }

                if (stopLoop) {
                    this.stop();
                    return;
                }


                if (gettingCoefficentsThroughUDP) {
                    motorSpeed = pidClass.calculateOutput(centimeters, distance, true);
                }
                else {
                    motorSpeed = pidClass.calculateOutput(centimeters, distance);
                }

                if (rangeSensor == RANGE_SENSORS.FRONT_RANGE_SENSOR) {
                    opMode.robot.drive.moveAtPower(-motorSpeed);
                }
                else if (rangeSensor == RANGE_SENSORS.REAR_RANGE_SENSOR) {
                    opMode.robot.drive.moveAtPower(motorSpeed);
                }

                opMode.robot.update();
            }
            this.stop();
        }

        @Override
        public void stop() {
            opMode.robot.drive.stopDriveMotors();
            opMode.robot.update();
        }
    };

    public Command getToWaypoint = new Command() {
        AutonomousMovement autonomousMovement;
        Waypoint targetPosition;

        double changeInX;
        double changeInY;
        double distanceToTravel;

        double whatToTurnTo;

        @Override
        public void run(CustomOpMode opMode, Object... inputs) {
            autonomousMovement = new AutonomousMovement();

            switch (inputs.length){
                case 0:
                    //If there are no arguments, throw exception
                    throw new IllegalArgumentException("Not Enough Arguments Given");
                case 1:
                    try {
                        this.targetPosition = (Waypoint) inputs[0];
                    } catch (ClassCastException e) {
                        throw new IllegalArgumentException("Argument(s) of the wrong type (Casting failed)");
                    }
                    break;
                default:
                    //If extra arguments are given, throw exception
                    throw new IllegalArgumentException("Too Many Arguments Given");
            }

            opMode.robot.update();

            changeInX = targetPosition.x - opMode.robot.drive.position.x;
            changeInY = targetPosition.y - opMode.robot.drive.position.y;
            distanceToTravel = Math.sqrt(Math.pow(changeInX, 2) + Math.pow(changeInY, 2));

            autonomousMovement.turn.run(opMode, Math.atan2(changeInY, changeInX));

            opMode.robot.update();

            this.loop();
        }

        @Override
        public void loop() {
            opMode.robot.drive.setAllMotorsToRelativePosition(1.0, convertFromCMToTicks(distanceToTravel));

            while(!opMode.robot.drive.areMotorsBusy){
                opMode.robot.update();
            }
            this.stop();
        }

        @Override
        public void stop() {
            opMode.robot.drive.position = targetPosition;
            opMode.robot.drive.motorRunMode = DcMotorEx.RunMode.RUN_USING_ENCODER;
            opMode.robot.drive.stopDriveMotors();
            opMode.robot.update();
        }
    };
}
