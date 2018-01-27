package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.hardware.*;
import static org.firstinspires.ftc.teamcode.hardware.jewelColorSensor;

/**
 * Created by dansm on 1/25/2018.
 */

@Autonomous(name="Read Distance")

public class backUpShowReadingsRevRangeSensor extends LinearOpMode{
    double distance;

    public void runOpMode() throws InterruptedException{

        initHardwareMap(hardwareMap);
        functions.initServos(false);

        waitForStart();

        while(opModeIsActive()){
            distance = sideRangeSensor.cmUltrasonic();
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        
    }
}
