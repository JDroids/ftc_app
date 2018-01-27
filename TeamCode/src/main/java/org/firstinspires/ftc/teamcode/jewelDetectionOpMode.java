package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by dansm on 1/21/2018.
 */

@Autonomous(name="Jewel Detection OpenCV")

public class jewelDetectionOpMode extends LinearOpMode{
    public void runOpMode(){
        jewelDetectionOpenCV jewelVision = new jewelDetectionOpenCV();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        jewelVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        jewelVision.enable();

        waitForStart();

        while(opModeIsActive()){

            if(!opModeIsActive()){
                break;
            }
        }
        jewelVision.disable();
    }
}
