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
    DigitalChannel firstLiftTopSwitch;
    DigitalChannel firstLiftBottomSwitch;
    DigitalChannel secondLiftTopSwitch;
    DigitalChannel secondLiftBottomSwitch;

    DcMotorEx firstGlyphLift;
    DcMotorEx secondGlyphLift;

    /**
     * An enum that defines various states that the glyph lift motors could be in
     */

    public enum GLYPH_LIFT_STATES {
        STOP,
        UP,
        DOWN
    }

    GLYPH_LIFT_STATES firstGlyphLiftState;
    GLYPH_LIFT_STATES secondGlyphLiftState;

    public GLYPH_LIFT_STATES getFirstGlyphLiftState() {
        return this.firstGlyphLiftState;
    }
    public void setFirstGlyphLiftState(GLYPH_LIFT_STATES firstGlyphLiftState) {
        this.firstGlyphLiftState = firstGlyphLiftState;
    }

    public GLYPH_LIFT_STATES getSecondGlyphLiftState() {
        return this.secondGlyphLiftState;
    }
    public void setSecondGlyphLiftState(GLYPH_LIFT_STATES secondGlyphLiftState) {
        this.secondGlyphLiftState = secondGlyphLiftState;
    }

    /**
     * Initializes the hardware of the glyph lifts
     *
     * This hardware includes
     * <ul>
     *     <li>DcMotorEx firstGlyphLift</li>
     *     <li>DcMotorEx secondGlyphLift</li>
     *
     *     <li>DigitalChannel firstLiftTopSwitch</li>
     *     <li>DigitalChannel firstLiftBottomSwitch</li>
     *     <li>DigitalChannel secondLiftTopSwitch</li>
     *     <li>DigitalChannel secondLiftBottomSwitch</li>
     * </ul>
     */

    public void initHardware(CustomOpMode opMode) {
        firstGlyphLift = opMode.hardwareMap.get(DcMotorEx.class, "MotorGlyphLift");
        secondGlyphLift = opMode.hardwareMap.get(DcMotorEx.class, "MotorGlyphLift2");

        firstLiftTopSwitch = opMode.hardwareMap.get(DigitalChannel.class, "FirstLiftSwitchUpper");
        firstLiftBottomSwitch = opMode.hardwareMap.get(DigitalChannel.class, "FirstLiftSwitchLower");

        secondLiftTopSwitch = opMode.hardwareMap.get(DigitalChannel.class, "SecondLiftSwitchUpper");
        secondLiftBottomSwitch = opMode.hardwareMap.get(DigitalChannel.class, "SecondLiftSwitchLower");

    }

    /**
     * Updates the motor power of the lift motors based on what firstGlyphLiftState and secondGlyphLiftState is set to
     *
     * This takes into account the value of the lift switches, meaning that the lifts won't go over the limit even if the states are not set to STOP
     */

    public void update() {
        if (firstLiftTopSwitch.getState() && firstGlyphLiftState == GLYPH_LIFT_STATES.DOWN) {
            firstGlyphLift.setPower(-1);
        }
        else if (firstLiftBottomSwitch.getState() && firstGlyphLiftState == GLYPH_LIFT_STATES.UP) {
            firstGlyphLift.setPower(1);
        }
        else {
            firstGlyphLift.setPower(0);
        }

        if (secondLiftTopSwitch.getState() && secondGlyphLiftState == GLYPH_LIFT_STATES.DOWN) {
            secondGlyphLift.setPower(-0.8);
        }
        else if (secondLiftBottomSwitch.getState() && secondGlyphLiftState == GLYPH_LIFT_STATES.UP) {
            secondGlyphLift.setPower(0.8);
        }
        else {
            secondGlyphLift.setPower(0);
        }
    }
}
