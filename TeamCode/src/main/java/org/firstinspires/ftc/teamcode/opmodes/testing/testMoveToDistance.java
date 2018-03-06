package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.functions.moveToDistanceUltrasonicPID;
import static org.firstinspires.ftc.teamcode.resources.hardware.frontRangeSensor;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;


/**
 * Created by dansm on 3/3/2018.
 */

@Autonomous(name = "HowToGiveNarmadaAHeartAttack")

public class testMoveToDistance extends LinearOpMode {
    public void runOpMode() {
        initHardwareMap(hardwareMap);

        waitForStart();

        moveToDistanceUltrasonicPID(frontRangeSensor, 30, this);
    }
}
