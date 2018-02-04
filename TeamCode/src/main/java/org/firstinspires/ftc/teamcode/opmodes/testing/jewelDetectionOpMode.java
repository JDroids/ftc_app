package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.resources.jewelDetectionOpenCV;

/**
 * Created by dansm on 1/21/2018.
 */

@Autonomous(name="Jewel Detection Custom")

public class jewelDetectionOpMode extends LinearOpMode{
    public void runOpMode(){
        jewelDetectionOpenCV jewelVision = new jewelDetectionOpenCV();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        jewelVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);

        // start the vision system
        jewelVision.enable();

        waitForStart();

        while(opModeIsActive()){
            switch(jewelVision.jewelOnLeft){
                case RED:
                    telemetry.addData("Jewel On Left", "Red");
                    break;
                case BLUE:
                    telemetry.addData("Jewel On Left", "Blue");
                    break;
                case NONE:
                    telemetry.addData("Jewel On Left", "None");
                    break;
            }
            if(isStopRequested()){
                jewelVision.disable();
            }
            telemetry.update();
        }
        jewelVision.disable();
    }
}
