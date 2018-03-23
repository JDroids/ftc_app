package org.firstinspires.ftc.teamcode.roboutils.templates;

/**
 * Created by dansm on 3/22/2018.
 */

public abstract class Command {
    public CustomOpMode opMode;

    public abstract void run(CustomOpMode opMode, Object... inputs);

    public abstract void loop();

    public abstract void stop();
}
