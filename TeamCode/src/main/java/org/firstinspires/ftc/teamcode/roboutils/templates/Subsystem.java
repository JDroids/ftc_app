package org.firstinspires.ftc.teamcode.roboutils.templates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Represents a physical subsystem on the robot
 */

public abstract class Subsystem {

    CustomOpMode opMode;

    /**
     * run control code (e.g., read sensors and update motors).
     */

    public abstract void update();

    /**
     * Used to init the hardware of each subsystem
     */
    public abstract void initHardware(CustomOpMode opMode);

}
