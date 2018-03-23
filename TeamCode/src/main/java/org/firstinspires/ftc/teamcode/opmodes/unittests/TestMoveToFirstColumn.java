package org.firstinspires.ftc.teamcode.opmodes.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.resources.constants;

import static org.firstinspires.ftc.teamcode.resources.functions.moveToFirstCryptoColumn;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;

/**
 * Created by dansm on 3/11/2018.
 */

@Disabled
@Autonomous(name = "Move To First Column Backwards")

public class TestMoveToFirstColumn extends LinearOpMode {
    public void runOpMode() {
        initHardwareMap(hardwareMap);

        waitForStart();

        moveToFirstCryptoColumn(constants.DIRECTION.BACKWARDS, this);
    }
}
