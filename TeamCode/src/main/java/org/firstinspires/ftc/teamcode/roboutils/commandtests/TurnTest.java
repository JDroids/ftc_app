package org.firstinspires.ftc.teamcode.roboutils.commandtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands.AutonomousMovement;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

/**
 * Created by dansm on 3/23/2018.
 */

@Autonomous(name="Turn Test", group="CommandTests")

public class TurnTest extends CustomOpMode {
    public void runOpMode() {
        robot = new RelicRecoveryRobot(this);
        robot.initHardware(this);

        waitForStart();

        robot.initServosForAutonomous();
        robot.update();

        AutonomousMovement autonomousMovement = new AutonomousMovement();

        autonomousMovement.turn.run(this, 90);

        autonomousMovement.turn.run(this, 0);
    }
}
