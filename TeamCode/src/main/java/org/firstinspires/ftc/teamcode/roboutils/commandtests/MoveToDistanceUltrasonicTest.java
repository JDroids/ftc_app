package org.firstinspires.ftc.teamcode.roboutils.commandtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands.AutonomousMovement;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

/**
 * Created by dansm on 3/23/2018.
 */

@Autonomous(name="Move To Distance Ultrasonic Test", group="CommandTests")

public class MoveToDistanceUltrasonicTest extends CustomOpMode{
    public void runOpMode(){
        robot = new RelicRecoveryRobot(this);
        robot.initHardware(this);

        robot.initServosForAutonomous();
        robot.update();

        waitForStart();

        AutonomousMovement autonomousMovement = new AutonomousMovement();

        autonomousMovement.moveToDistanceUltrasonic.run(this, AutonomousMovement.RANGE_SENSORS.FRONT_RANGE_SENSOR, 50);
    }
}
