package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by dansm on 2/11/2018.
 */
@Disabled

@Autonomous(name="2 MR Range sensor test")
public class TwoUltrasonicSensors extends LinearOpMode{


    public void runOpMode(){
        ModernRoboticsI2cRangeSensor range1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range1");
        ModernRoboticsI2cRangeSensor range2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Range 1", range1.cmUltrasonic());
            telemetry.addData("Range 2", range2.cmUltrasonic());
            telemetry.update();
        }

    }
}
