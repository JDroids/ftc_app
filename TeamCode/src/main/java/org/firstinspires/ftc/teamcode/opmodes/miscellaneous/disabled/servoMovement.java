package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.hardware.glyphGrabberBL;
import static org.firstinspires.ftc.teamcode.resources.hardware.glyphGrabberBR;
import static org.firstinspires.ftc.teamcode.resources.hardware.glyphGrabberTL;
import static org.firstinspires.ftc.teamcode.resources.hardware.glyphGrabberTR;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.jewelArm;
import static org.firstinspires.ftc.teamcode.resources.hardware.jewelKnocker;

/**
 * Created by dansm on 12/22/2017.
 */
@Disabled
@TeleOp(name = "Check Servo Position")

public class servoMovement extends LinearOpMode {
    @Override
    public void runOpMode() {
        initHardwareMap(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
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
