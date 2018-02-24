package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import android.util.Log;

import com.disnodeteam.dogecv.ActivityViewDisplay;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.resources.constants;
import org.firstinspires.ftc.teamcode.resources.jewelDetectionOpenCV;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;

/**
 * Created by dansm on 1/21/2018.
 */
@Disabled
@Autonomous(name="Jewel Detection Custom")

public class jewelDetectionOpMode extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException{
        initHardwareMap(hardwareMap);
        initServos(constants.AUTONOMOUS);

        ArrayList<constants.JDColor> listOfJewelColors = new ArrayList<constants.JDColor>();

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

            if(listOfJewelColors.size() < 5){
                if(jewelVision.jewelOnLeft != constants.JDColor.NONE) {
                    listOfJewelColors.add(jewelVision.jewelOnLeft);
                }
            }
            else{
                break;
            }
        }

        jewelVision.disable();

        int redJewelsFound = 0;
        int blueJewelsFound = 0;

        for(constants.JDColor color : listOfJewelColors){
            if(color == constants.JDColor.RED){
                redJewelsFound++;
            }
            else{
                blueJewelsFound++;
            }
        }

        lowerJewelArms(this);

        int certainty = 0;

        constants.JDColor jewelOnRight = detectJewelColor(this);

        //We assume we are on the blue side

        //Knock Jewel takes the color of the jewel on the RIGHT side, which is what we detect with the color sensor, but with OpenCV we detect the LEFT one
        if(blueJewelsFound >= 4 && jewelOnRight == constants.JDColor.RED){
            certainty = blueJewelsFound * 20;

            telemetry.addData("Jewel On Left", "Blue");
            telemetry.addData("Certainty", certainty);
            Log.d("JewelOnLeft", "Blue");
            Log.d("Certainty", Integer.toString(certainty));

            knockJewel(constants.JDColor.RED, constants.JDColor.BLUE, this);
        }
        else if(redJewelsFound >= 4 && jewelOnRight == constants.JDColor.BLUE){
            certainty = redJewelsFound * 20;

            telemetry.addData("Jewel On Left", "Red");
            telemetry.addData("Certainty", certainty);
            Log.d("JewelOnLeft", "Red");
            Log.d("Certainty", Integer.toString(certainty));

            knockJewel(constants.JDColor.BLUE, constants.JDColor.BLUE, this);
        }
        else{
            telemetry.addData("Jewel On Left", "Unclear");
            Log.d("JewelOnLeft", "Unknown");

            knockJewel(jewelOnRight, constants.JDColor.BLUE, this);
        }

        telemetry.update();
        sleep(2000);
    }
}
