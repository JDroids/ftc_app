package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.opmodes.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands.AutonomousMovement;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

/**
 * Created by dansm on 3/23/2018.
 */

@Disabled
@Autonomous(name="Move To Distance Ultrasonic Test", group="Tests")

public class MoveToDistanceUltrasonicTest extends CustomOpMode{
    public void runOpMode(){
        robot = new RelicRecoveryRobot(this, new Waypoint(0, 0));
        robot.initHardware(this);

        robot.initServosForTeleop();
        robot.update();

        sleep(100);

        waitForStart();

        robot.grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.CLOSED;
        robot.grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.CLOSED;

        robot.update();

        sleep(100);

        AutonomousMovement autonomousMovement = new AutonomousMovement();

        autonomousMovement.moveToDistanceUltrasonic.run(this, AutonomousMovement.RANGE_SENSORS.FRONT_RANGE_SENSOR, 50);
    }
}
