package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

/**
 * Created by dansm on 3/21/2018.
 */

public class Grabber extends Subsystem {
    static public Servo glyphGrabberTL = null;
    static public Servo glyphGrabberTR = null;
    static public Servo glyphGrabberBL = null;
    static public Servo glyphGrabberBR = null;

    public enum GRABBER_POSITIONS {
        INIT,
        TELEOP_INIT,
        WIDE_OPEN,
        OPEN,
        CLOSED
    }

    public GRABBER_POSITIONS topGrabberPosition;
    public GRABBER_POSITIONS bottomGrabberPosition;

    public void initHardware(CustomOpMode opMode) {
        //The top and bottom servos are flipped in the config, this is fixed with the variable names
        glyphGrabberTL = opMode.hardwareMap.get(Servo.class, "glyphGrabberBL");
        glyphGrabberTR = opMode.hardwareMap.get(Servo.class, "glyphGrabberBR");
        glyphGrabberBL = opMode.hardwareMap.get(Servo.class, "glyphGrabberTL");
        glyphGrabberBR = opMode.hardwareMap.get(Servo.class, "glyphGrabberTR");
        this.update();
    }

    public void update() {
        if (topGrabberPosition == GRABBER_POSITIONS.INIT) {
            glyphGrabberTL.setPosition(0.9);
            glyphGrabberTR.setPosition(0.3);
        } else if (topGrabberPosition == GRABBER_POSITIONS.TELEOP_INIT || topGrabberPosition == GRABBER_POSITIONS.WIDE_OPEN) {
            glyphGrabberTL.setPosition(0.5);
            glyphGrabberTR.setPosition(0.65);
        } else if (topGrabberPosition == GRABBER_POSITIONS.OPEN) {
            glyphGrabberTL.setPosition(0.35);
            glyphGrabberTR.setPosition(0.7);
        } else if (topGrabberPosition == GRABBER_POSITIONS.CLOSED) {
            glyphGrabberTL.setPosition(0.25);
            glyphGrabberTR.setPosition(1.0);
        }

        if (bottomGrabberPosition == GRABBER_POSITIONS.INIT) {
            glyphGrabberBL.setPosition(1.0);
            glyphGrabberBR.setPosition(0.0);
        } else if (bottomGrabberPosition == GRABBER_POSITIONS.TELEOP_INIT || bottomGrabberPosition == GRABBER_POSITIONS.WIDE_OPEN) {
            glyphGrabberBL.setPosition(0.55);
            glyphGrabberBR.setPosition(0.2);
        } else if (bottomGrabberPosition == GRABBER_POSITIONS.OPEN) {
            glyphGrabberBL.setPosition(0.45);
            glyphGrabberBR.setPosition(0.25);
        } else if (bottomGrabberPosition == GRABBER_POSITIONS.CLOSED) {
            glyphGrabberBL.setPosition(0.25);
            glyphGrabberBR.setPosition(0.4);
        }


    }
}
