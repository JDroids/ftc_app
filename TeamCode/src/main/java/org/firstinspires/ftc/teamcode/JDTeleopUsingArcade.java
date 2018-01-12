package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.functions.*;
import static org.firstinspires.ftc.teamcode.hardware.*;



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

        double gamepad1LeftY;
        double gamepad1RightY;
        double gamepad1LeftX;
        double gamepad1RightX;

        waitForStart();
        //Code to run after play is pressed

        initServos();

        while(opModeIsActive()) {
            gamepad1LeftY = scaleInputFixedSpeed(gamepad1.left_stick_y);
            gamepad1LeftX = scaleInputFixedSpeed(gamepad1.left_stick_x);
            gamepad1RightX = scaleInputFixedSpeed(gamepad1.right_stick_x);

            moveArcade(gamepad1);

            if (gamepad2.a) {
                closeGrabber(constants.BOTH_GRABBERS);
            } else if (gamepad2.b) {
                openGrabber(constants.BOTH_GRABBERS);
            } else if (gamepad2.y) {
                openGrabberWide(constants.BOTH_GRABBERS);
            }

            firstLift(gamepad2, this);
            secondLift(gamepad2, this);
        }
    }
}
