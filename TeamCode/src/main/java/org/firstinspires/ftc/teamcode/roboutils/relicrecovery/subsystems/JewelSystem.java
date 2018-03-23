package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

/**
 * Created by dansm on 3/21/2018.
 */

public class JewelSystem extends Subsystem {
    public enum JEWEL_KNOCKER_POSITIONS {
        INIT,
        KNOCK_LEFT_JEWEL,
        KNOCK_RIGHT_JEWEL,
        INBETWEEN
    }

    public JEWEL_KNOCKER_POSITIONS jewelKnockerPosition;

    public enum JEWEL_ARM_POSITIONS {
        INIT,
        UP,
        DOWN
    }

    public JEWEL_ARM_POSITIONS jewelArmPosition;

    Servo jewelKnocker;
    Servo jewelArm;

    public void initHardware(CustomOpMode opMode) {
        jewelKnocker = opMode.hardwareMap.get(Servo.class, "servoJewelKnock");
        jewelArm = opMode.hardwareMap.get(Servo.class, "servoJewelArm");
    }

    @Override
    public void update() {
        if (jewelArmPosition == JEWEL_ARM_POSITIONS.INIT || jewelArmPosition == JEWEL_ARM_POSITIONS.UP) {
            jewelArm.setPosition(0.9);
        } else if (jewelArmPosition == JEWEL_ARM_POSITIONS.DOWN) {
            jewelArm.setPosition(0);
        }

        if (jewelKnockerPosition == JEWEL_KNOCKER_POSITIONS.INIT || jewelKnockerPosition == JEWEL_KNOCKER_POSITIONS.KNOCK_LEFT_JEWEL) {
            jewelKnocker.setPosition(0);
        } else if (jewelKnockerPosition == JEWEL_KNOCKER_POSITIONS.KNOCK_RIGHT_JEWEL) {
            jewelKnocker.setPosition(1);
        } else if (jewelKnockerPosition == JEWEL_KNOCKER_POSITIONS.INBETWEEN) {
            jewelKnocker.setPosition(0);
        }
    }
}
