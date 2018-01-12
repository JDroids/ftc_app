package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by dansm on 12/6/2017.
 */


@TeleOp(name = "Hello World")
@Disabled

public class HelloWorld extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException{
        telemetry.addData("Message", "Hello Nerd!");
        telemetry.addData("Message", "Hello World!");
        telemetry.update();

        //Code to run after init is pressed
        waitForStart();
        int i = 0;
        while(opModeIsActive()){
            telemetry.addData("Approximate Amount of Seconds Passed:", i);
            telemetry.update();
            Thread.sleep(1000);
            i+=1;
        }
        //Code to run after play is pressed
    }
}
