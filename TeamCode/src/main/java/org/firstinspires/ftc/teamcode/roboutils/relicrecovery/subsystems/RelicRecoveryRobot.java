package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

/**
 * Created by dansm on 3/21/2018.
 */

public class RelicRecoveryRobot extends Subsystem {
    public MecanumDrive drive;
    public JewelSystem jewelSystem;
    public Grabber grabber;
    public RelicRecoverer relicRecoverer;
    public GlyphLifts glyphLifts;

    CustomOpMode opMode;

    public RelicRecoveryRobot(CustomOpMode opMode, Waypoint startingPosition) {
        this.opMode = opMode;

        this.drive = new MecanumDrive();
        this.jewelSystem = new JewelSystem();
        this.grabber = new Grabber();
        this.relicRecoverer = new RelicRecoverer();
        this.glyphLifts = new GlyphLifts();

        this.drive.position = startingPosition;

        this.initHardware(this.opMode);
    }

    public void initHardware(CustomOpMode opMode) {
        this.drive.initHardware(this.opMode);
        this.jewelSystem.initHardware(this.opMode);
        this.grabber.initHardware(this.opMode);
        this.relicRecoverer.initHardware(this.opMode);
        this.glyphLifts.initHardware(this.opMode);
    }

    public void initServosForAutonomous() {
        this.jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        this.jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        this.grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;
        this.grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;

        this.relicRecoverer.relicExtensionServoPosition = RelicRecoverer.RELIC_EXTENSION_SERVO_POSITION.INIT;
        this.relicRecoverer.relicRotationalServoPosition = 1.0;

        this.drive.moveAtPower(0);
    }

    public void initServosForTeleop() {
        this.jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        this.jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        this.grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;
        this.grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;

        this.relicRecoverer.relicExtensionServoPosition = RelicRecoverer.RELIC_EXTENSION_SERVO_POSITION.INIT;
        this.relicRecoverer.relicRotationalServoPosition = 1.0;

        this.glyphLifts.firstGlyphLiftState = GlyphLifts.GLYPH_LIFT_STATES.STOP;
        this.glyphLifts.secondGlyphLiftState = GlyphLifts.GLYPH_LIFT_STATES.STOP;

        this.drive.motorSpeeds.add(0.0);
        this.drive.motorSpeeds.add(0.0);
        this.drive.motorSpeeds.add(0.0);
        this.drive.motorSpeeds.add(0.0);
    }

    @Override
    public void update() {
        this.drive.update();
        this.jewelSystem.update();
        this.grabber.update();
        this.glyphLifts.update();
        this.relicRecoverer.update();

        this.opMode.universalLog("Robot Position", (int) this.drive.position.x + ", " + (int) this.drive.position.y);

        this.opMode.universalLog("Front Range Sensor Distance", Double.toString(this.drive.frontRangeSensorDistance));
        this.opMode.universalLog("Side Range Sensor Distance", Double.toString(this.drive.sideRangeSensorDistance));
        this.opMode.universalLog("Rear Range Sensor Distance", Double.toString(this.drive.rearRangeSensorDistance));

        this.opMode.universalLog("Heading", Double.toString(drive.heading));

        this.opMode.universalLog("Front Left Motor Power", Double.toString(this.drive.motorSpeeds.get(0)));
        this.opMode.universalLog("Front Right Motor Power", Double.toString(this.drive.motorSpeeds.get(1)));
        this.opMode.universalLog("Back Left Motor Power", Double.toString(this.drive.motorSpeeds.get(2)));
        this.opMode.universalLog("Back Right Motor Power", Double.toString(this.drive.motorSpeeds.get(3)));

        this.opMode.universalLog("Relic Extension Servo Position", this.relicRecoverer.relicExtensionServoPosition.name());
        this.opMode.universalLog("Relic Rotational Servo Position", Double.toString(this.relicRecoverer.relicRotationalServoPosition));

        this.opMode.universalLog("Top Grabber Position", this.grabber.topGrabberPosition.name());
        this.opMode.universalLog("Bottom Grabber Position", this.grabber.bottomGrabberPosition.name());

        this.opMode.universalLog("Jewel Arm Position", this.jewelSystem.jewelArmPosition.name());
        this.opMode.universalLog("Jewel Knocker Position", this.jewelSystem.jewelKnockerPosition.name());


        this.opMode.telemetry.update();
    }

}
