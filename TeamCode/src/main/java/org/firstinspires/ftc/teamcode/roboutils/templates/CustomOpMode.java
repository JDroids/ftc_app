package org.firstinspires.ftc.teamcode.roboutils.templates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;

/**
 * Created by dansm on 3/22/2018.
 * An expansion of LinearOpMode
 */

public abstract class CustomOpMode extends LinearOpMode {
    /**
     * Used for easily accessing the robot in various places
     */

    public RelicRecoveryRobot robot;

    /**
     * Logs something to both Logcat (at level debug) and telemetry
     *
     * The tag for the thing logged to Logcat is "JDLog"
     * Telemetry is not updated within this function, that has to be done externally
     */

    public void universalLog(String caption, String content) {
        telemetry.addData(caption, content);
        Log.d("JDLog", caption + ": " + content);
    }
}
