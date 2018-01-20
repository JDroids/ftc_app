package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.functions.*;

/**
 * Created by dansm on 1/14/2018.
 */
@Disabled
@Autonomous(name="Detect Jewels")
public class readJewelsWithDogeCV extends LinearOpMode{
    @Override

    public void runOpMode(){
        JewelDetector jewelDetector = initDogeCVForJewel(hardwareMap);
        waitForStart();

        readJewelWithDogeCV(jewelDetector, this);


    }
}
