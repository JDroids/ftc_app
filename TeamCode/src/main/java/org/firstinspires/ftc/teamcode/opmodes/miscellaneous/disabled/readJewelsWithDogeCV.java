package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.functions.initDogeCVForJewel;
import static org.firstinspires.ftc.teamcode.resources.functions.readJewelsWithDogeCV;

/**
 * Created by dansm on 1/14/2018.
 */
@Disabled
@Autonomous(name = "Detect Jewels")
public class readJewelsWithDogeCV extends LinearOpMode {
    @Override

    public void runOpMode() {
        JewelDetector jewelDetector = initDogeCVForJewel(hardwareMap);
        waitForStart();

        readJewelsWithDogeCV(jewelDetector, this);


    }
}
