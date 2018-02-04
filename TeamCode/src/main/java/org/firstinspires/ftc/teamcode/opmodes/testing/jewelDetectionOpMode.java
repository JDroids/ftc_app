package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.resources.jewelDetectionOpenCV;
/**
 * Created by dansm on 1/21/2018.
 */

@Autonomous(name="Jewel Detection Custom")

public class jewelDetectionOpMode extends LinearOpMode{
    @Override

    public void runOpMode(){
        jewelDetectionOpenCV jewelVision = new jewelDetectionOpenCV();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        jewelVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);

        // start the vision system
        jewelVision.enable();

        ElapsedTime mRuntime = new ElapsedTime();

        waitForStart();
        mRuntime.reset();


        while(opModeIsActive()){
            telemetry.addData("Jewel On Left", jewelVision.jewelOnLeft);
            telemetry.addData("Time Elapsed", mRuntime.milliseconds());
            telemetry.update();
        }
        jewelVision.disable();
    }
}
