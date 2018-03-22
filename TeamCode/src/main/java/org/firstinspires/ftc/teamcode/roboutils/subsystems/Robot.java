package org.firstinspires.ftc.teamcode.roboutils.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by dansm on 3/21/2018.
 */

//TODO: Implement relic stuff, glyph lifts

public class Robot extends Subsystem{
    FourWheelDriveTrain driveTrain;
    JewelSystem jewelSystem;
    Grabber grabber;

    public Robot(OpMode opMode){
        initRobotHardware(opMode.hardwareMap);
    }
    
    void initRobotHardware(HardwareMap hardwareMap){
        driveTrain.driveTrainType = FourWheelDriveTrain.DRIVE_TRAIN_TYPE.MECANUM;
        driveTrain.initFourWheelDriveTrain(hardwareMap);

        jewelSystem.initJewelKnocker(hardwareMap);
        grabber.initGrabber(hardwareMap);
    }

    public void initServosForAutonomous(){
        jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;
        grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.INIT;

        update();
    }

    public void initServosForTeleop(){
        jewelSystem.jewelArmPosition = JewelSystem.JEWEL_ARM_POSITIONS.INIT;
        jewelSystem.jewelKnockerPosition = JewelSystem.JEWEL_KNOCKER_POSITIONS.INIT;

        grabber.topGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;
        grabber.bottomGrabberPosition = Grabber.GRABBER_POSITIONS.TELEOP_INIT;

        update();
    }

    @Override
    public void update(){
        driveTrain.update();
        jewelSystem.update();
        grabber.update();
    }

}
