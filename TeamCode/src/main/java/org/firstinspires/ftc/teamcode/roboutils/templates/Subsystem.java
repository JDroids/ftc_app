package org.firstinspires.ftc.teamcode.roboutils.templates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by dansm on 3/21/2018.
 */

public abstract class Subsystem {

    /**
     * run control code (e.g., read sensors and update motors) and add telemetry.
     */

    public abstract void update();

    public abstract void initHardware(CustomOpMode opMode);

}
