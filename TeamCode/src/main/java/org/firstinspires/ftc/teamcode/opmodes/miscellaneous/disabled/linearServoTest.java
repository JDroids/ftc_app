package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.functions.initServos;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicExtensionServo;

/**
 * Created by dansm on 2/23/2018.
 */

@Disabled
@TeleOp(name = "Linear Servo test")

public class linearServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap(hardwareMap);

        waitForStart();

        initServos(true);
        /*
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) { //To open
                if (relicExtensionServo.getPosition() < 0.9) {
                    relicExtensionServo.setPosition(relicExtensionServo.getPosition() + 0.01);
                }
            } else if (gamepad1.left_bumper) { //To close
                if (relicExtensionServo.getPosition() > 0.3) {
                    relicExtensionServo.setPosition(relicExtensionServo.getPosition() - 0.01);
                }
            } else if (gamepad1.a) {
                relicExtensionServo.setPosition(0.5);
            } else if (gamepad1.b) {
                relicExtensionServo.setPosition(0.8);
            }
        }*/

    }
}
