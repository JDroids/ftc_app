package org.firstinspires.ftc.teamcode.roboutils.templates;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;

/**
 * Created by dansm on 3/22/2018.
 * An abstract class that is used so implementations of the Command class {@link Command} can access the running CustomOpMode's robot
 */

public abstract class CustomOpMode extends LinearOpMode {
    public RelicRecoveryRobot robot;

    public void universalLog(String caption, String content){
        this.telemetry.addData(caption, content);
        Log.d("JDLog", caption + content);
    }
}
