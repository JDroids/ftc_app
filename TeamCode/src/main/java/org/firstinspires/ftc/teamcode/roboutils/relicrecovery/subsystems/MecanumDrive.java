package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboutils.templates.DriveTrain;
import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

import java.util.ArrayList;

/**
 * Created by dansm on 3/21/2018.
 */

public class MecanumDrive extends DriveTrain {
    ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>();

    public void initHardware(OpMode opMode){
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "FrontRight"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackLeft"));
        motors.add(opMode.hardwareMap.get(DcMotorEx.class, "BackRight"));

        for(DcMotorEx motor : motors){
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }
    }

}
