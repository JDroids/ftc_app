package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

/**
 * Created by dansm on 3/22/2018.
 */

public class GlyphLifts extends Subsystem {
    public DigitalChannel firstLiftTopSwitch;
    public DigitalChannel firstLiftBottomSwitch;
    public DigitalChannel secondLiftTopSwitch;
    public DigitalChannel secondLiftBottomSwitch;

    public DcMotorEx firstGlyphLift;
    public DcMotorEx secondGlyphLift;

    public enum GLYPH_LIFT_STATES {
        STOP,
        UP,
        DOWN
    }

    GLYPH_LIFT_STATES firstGlyphLiftState;
    GLYPH_LIFT_STATES secondGlyphLiftState;

    public void initHardware(CustomOpMode opMode) {
        firstGlyphLift = opMode.hardwareMap.get(DcMotorEx.class, "MotorGlyphLift");
        secondGlyphLift = opMode.hardwareMap.get(DcMotorEx.class, "MotorGlyphLift2");

        firstLiftTopSwitch = opMode.hardwareMap.get(DigitalChannel.class, "FirstLiftSwitchUpper");
        firstLiftBottomSwitch = opMode.hardwareMap.get(DigitalChannel.class, "FirstLiftSwitchLower");

        secondLiftTopSwitch = opMode.hardwareMap.get(DigitalChannel.class, "SecondLiftSwitchUpper");
        secondLiftBottomSwitch = opMode.hardwareMap.get(DigitalChannel.class, "SecondLiftSwitchLower");

    }

    public void update() {
        if (firstLiftTopSwitch.getState() && firstGlyphLiftState == GLYPH_LIFT_STATES.DOWN) {
            firstGlyphLift.setPower(-1);
        } else if (firstLiftBottomSwitch.getState() && firstGlyphLiftState == GLYPH_LIFT_STATES.UP) {
            firstGlyphLift.setPower(1);
        } else {
            firstGlyphLift.setPower(0);
        }

        if (secondLiftTopSwitch.getState() && secondGlyphLiftState == GLYPH_LIFT_STATES.DOWN) {
            secondGlyphLift.setPower(-0.8);
        } else if (secondLiftBottomSwitch.getState() && secondGlyphLiftState == GLYPH_LIFT_STATES.UP) {
            secondGlyphLift.setPower(0.8);
        } else {
            secondGlyphLift.setPower(0);
        }
    }
}
