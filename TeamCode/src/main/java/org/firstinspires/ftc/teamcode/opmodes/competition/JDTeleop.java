package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;
import static org.firstinspires.ftc.teamcode.resources.constants.*;


/**
 * Created by dansm on 12/7/2017.
 */

@TeleOp(name="JDTeleOp")

public class JDTeleop extends LinearOpMode{

    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState;

        initHardwareMap(hardwareMap);

        waitForStart();
        //Code to run after play is pressed

        GRABBERS controlledGrabbers = GRABBERS.BOTH_GRABBERS;

        initServos(TELEOP);

        moveFirstLiftForTime(0.7, 500, this);
        moveSecondLiftForTime(0.7, 500, this);

        while(opModeIsActive()) {
            moveArcade(gamepad1);

            //To change which set of grabbers should be used
            if(gamepad2.dpad_up){
                controlledGrabbers = GRABBERS.TOP_GRABBER;
            }
            else if(gamepad2.dpad_down){
                controlledGrabbers = GRABBERS.BOTTOM_GRABBER;
            }
            else if(gamepad2.dpad_left || gamepad2.dpad_right){
                controlledGrabbers = GRABBERS.BOTH_GRABBERS;
            }

            //To open/close grabber
            if (gamepad2.a) {
                closeGrabber(controlledGrabbers);
            }
            else if (gamepad2.b){
                openGrabber(controlledGrabbers);
            }
            else if (gamepad2.y) {
                openGrabberWide(controlledGrabbers);
            }

            //To extend/detract cascading rail
            if(gamepad2.right_bumper){
                relicExtender.setPower(0.5);
            }
            else if(gamepad2.left_bumper){
                relicExtender.setPower(-0.3);
            }
            else{
                relicExtender.setPower(0);
            }

            //To extend/detract the linear servo on the relic mechanism
            if(gamepad1.right_bumper){
                if(relicLinearServo.getPosition() < 0.7){
                    relicLinearServo.setPosition(relicLinearServo.getPosition() + 0.001);
                }
            }
            else if(gamepad1.left_bumper){
                if(relicLinearServo.getPosition() > 0.3){
                    relicLinearServo.setPosition(relicLinearServo.getPosition() - 0.001);
                }
            }

            //To move the rotational servo on the relic mechanism
            if(gamepad1.a){
                relicRotationalServo.setPosition(relicRotationalServo.getPosition() + 0.001);
            }
            else if(gamepad1.b){
                relicRotationalServo.setPosition(relicRotationalServo.getPosition() - 0.001);
            }

            controlFirstGlyphLift(gamepad2, this);
            controlSecondGlyphLift(gamepad2, this);
        }
    }
}
