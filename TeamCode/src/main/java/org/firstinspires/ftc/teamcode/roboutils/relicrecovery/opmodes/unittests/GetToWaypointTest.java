package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.opmodes.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.commands.AutonomousMovement;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

import java.util.ArrayList;

/**
 * Created by dansm on 3/27/2018.
 */

@Autonomous(name="GetToWaypointTest", group="Tests")
public class GetToWaypointTest extends CustomOpMode {

    public void runOpMode() {
        robot = new RelicRecoveryRobot(this, new Waypoint(0, 0));

        robot.initHardware(this);

        robot.initServosForTeleop();

        robot.update();

        waitForStart();

        AutonomousMovement autonomousMovement = new AutonomousMovement();

        ArrayList<Waypoint> waypointList = new ArrayList<>();

        waypointList.add(new Waypoint(30, 0));
        waypointList.add(new Waypoint(30, 30));
        waypointList.add(new Waypoint(0, 30));
        waypointList.add(new Waypoint(0, 0));

        for (Waypoint waypoint : waypointList) {
            autonomousMovement.getToWaypoint.run(this, waypoint);
        }
    }
}
