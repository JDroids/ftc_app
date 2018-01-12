package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.*;

/**
 * Created by dansm on 12/22/2017.
 */

@TeleOp(name="Check Servo Position")

public class servoMovement extends LinearOpMode{
    @Override
    public void runOpMode(){
        initHardwareMap(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("TL Servo", glyphGrabberTL.getPosition());
            telemetry.addData("TR Servo", glyphGrabberTR.getPosition());
            telemetry.addData("BL Servo", glyphGrabberBL.getPosition());
            telemetry.addData("BR Servo", glyphGrabberBR.getPosition());
            telemetry.addData("Jewel Knocker", jewelKnocker.getPosition());
            telemetry.addData("Jewel Arm", jewelArm.getPosition());
            telemetry.update();
        }
    }
}
