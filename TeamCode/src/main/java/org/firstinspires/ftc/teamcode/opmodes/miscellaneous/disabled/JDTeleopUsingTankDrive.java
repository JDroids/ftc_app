package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.constants.*;
import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;


/**
 * Created by dansm on 12/7/2017.
 */


@Disabled
@TeleOp(name="JDTeleOpTankDrive")

public class JDTeleopUsingTankDrive extends LinearOpMode{
    @Override

    public void runOpMode() throws InterruptedException{
        //Code to run after init is pressed
        boolean hardwareMapState;

        initHardwareMap(hardwareMap);

        double gamepad1LeftY;
        double gamepad1RightY;
        double gamepad1LeftX;
        double gamepad1RightX;

        waitForStart();
        //Code to run after play is pressed

        initServos(true);

        while(opModeIsActive()){
            gamepad1LeftY = scaleInputFixedSpeed(gamepad1.left_stick_y);
            gamepad1RightY = scaleInputFixedSpeed(gamepad1.right_stick_y);
            gamepad1LeftX = scaleInputFixedSpeed(gamepad1.left_stick_x);
            gamepad1RightX = scaleInputFixedSpeed(gamepad1.right_stick_x);

            move(gamepad1LeftY, gamepad1RightY, gamepad1LeftX, gamepad1RightX);

            if(gamepad2.a){
                closeGrabber(GRABBERS.BOTH_GRABBERS);
            }
            else if(gamepad2.b){
                openGrabber(GRABBERS.BOTH_GRABBERS);
            }
            else if(gamepad2.y){
                openGrabberWide(GRABBERS.BOTH_GRABBERS);
            }

            controlFirstGlyphLift(gamepad2, this);
            controlSecondGlyphLift(gamepad2, this);
        }
    }
}
