package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;


/**
 * Created by dansm on 3/3/2018.
 */

@Autonomous(name="Test Move To Distance")

public class testMoveToDistance extends LinearOpMode{
    public void runOpMode(){
        initHardwareMap(hardwareMap);

        waitForStart();

        moveToDistanceUltrasonicPID(frontRangeSensor, 10, this, true);
    }
}
