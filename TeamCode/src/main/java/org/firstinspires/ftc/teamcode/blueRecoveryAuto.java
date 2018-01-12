package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.constants.*;
import static org.firstinspires.ftc.teamcode.functions.*;
import static org.firstinspires.ftc.teamcode.hardware.*;

/**
 * Created by dansm on 12/21/2017.
 */

@Autonomous(name="BLUERecoveryAuto")
public class blueRecoveryAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

        //Code to run after init is pressed

        initHardwareMap(hardwareMap);

        initServos();

        double distanceToWall = sideRangeSensor.cmUltrasonic();

        telemetry.addData("Distance to wall", distanceToWall);
        telemetry.update();

        waitForStart();

        //Code to run after play is pressed

        RelicRecoveryVuMark vuMark = getVumark(this, hardwareMap);

        if(vuMark == RelicRecoveryVuMark.LEFT){
            telemetry.addData("Column to Go For", "Left");
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER){
            telemetry.addData("Column to Go For", "Center");
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT){
            telemetry.addData("Column to Go For", "Right");
        }

        telemetry.update();

        sleep(2000);

        closeGrabber(BOTTOM_GRABBER);

        moveUntilCryptoWall(distanceToWall, vuMark, BLUE, this);


        turn(90, this);

        sleep(100);

        moveForTime(0.3, 3000, this);

        openGrabber(BOTTOM_GRABBER);
        moveForTime(-0.3, 1000, this);

        openGrabberWide(BOTTOM_GRABBER);

    }
}
