package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.functions.*;
import static org.firstinspires.ftc.teamcode.hardware.*;
import static org.firstinspires.ftc.teamcode.constants.*;


/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleOpJavaArcade")

public class JDTeleopUsingArcade extends LinearOpMode{

    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState;

        initHardwareMap(hardwareMap);

        waitForStart();
        //Code to run after play is pressed

        int controlledGrabber = BOTH_GRABBERS;

        initServos(TELEOP);

        moveLiftForTime(0.7, 500, this);
        moveSecondLiftForTime(0.7, 500, this);

        while(opModeIsActive()) {
            moveArcade(gamepad1);

            if(gamepad2.dpad_up){
                controlledGrabber = TOP_GRABBER;
            }
            else if(gamepad2.dpad_down){
                controlledGrabber = BOTTOM_GRABBER;
            }
            else if(gamepad2.dpad_left || gamepad2.dpad_right){
                controlledGrabber = BOTH_GRABBERS;
            }

            if (gamepad2.a) {
                closeGrabber(controlledGrabber);
            }
            else if (gamepad2.b){
                openGrabber(controlledGrabber);
            }
            else if (gamepad2.y) {
                openGrabberWide(controlledGrabber);
            }

            firstLift(gamepad2, this);
            secondLift(gamepad2, this);
        }
    }
}
