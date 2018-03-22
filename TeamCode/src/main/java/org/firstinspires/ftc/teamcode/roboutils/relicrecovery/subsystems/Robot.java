package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

/**
 * Created by dansm on 3/21/2018.
 */

//TODO: Implement relic stuff, glyph lifts

public class Robot extends Subsystem {
    MecanumDrive drive;
    JewelSystem jewelSystem;
    Grabber grabber;
    RelicRecoverer relicRecoverer;
    GlyphLifts glyphLifts;

    public Robot(OpMode opMode){
        initHardware(opMode);
    }
    
    public void initHardware(OpMode opMode){
        drive.initHardware(opMode);
        jewelSystem.initHardware(opMode);
        grabber.initHardware(opMode);
        relicRecoverer.initHardware(opMode);
        glyphLifts.initHardware(opMode);
    }

    public void initServosForAutonomous(){
        jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;
        grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;

        relicRecoverer.relicExtensionServoPosition = RelicRecoverer.RELIC_EXTENSION_SERVO_POSITION.INIT;
        relicRecoverer.relicRotationalServoPosition = 1.0;

        update();
    }

    public void initServosForTeleop(){
        jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;
        grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;

        relicRecoverer.relicExtensionServoPosition = RelicRecoverer.RELIC_EXTENSION_SERVO_POSITION.INIT;
        relicRecoverer.relicRotationalServoPosition = 1.0;

        glyphLifts.firstGlyphLiftState = GlyphLifts.GLYPH_LIFT_STATES.STOP;
        glyphLifts.secondGlyphLiftState = GlyphLifts.GLYPH_LIFT_STATES.STOP;

        update();
    }

    @Override
    public void update(){
        drive.update();
        jewelSystem.update();
        grabber.update();
        relicRecoverer.update();
    }

}
