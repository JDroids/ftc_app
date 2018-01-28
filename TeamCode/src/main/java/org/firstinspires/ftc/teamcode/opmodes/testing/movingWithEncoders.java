package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;

/**
 * Created by dansm on 1/27/2018.
 */

@Autonomous(name="moveEncoders")

public class movingWithEncoders extends LinearOpMode{

    @Override
    public void runOpMode(){

        initHardwareMap(hardwareMap);

        waitForStart();

        moveEncoders(-5, -0.7, this);

    }
}
