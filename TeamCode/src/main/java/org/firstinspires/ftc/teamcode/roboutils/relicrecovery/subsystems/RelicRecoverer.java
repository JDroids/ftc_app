package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboutils.templates.Subsystem;

import static org.firstinspires.ftc.teamcode.resources.hardware.relicExtensionServo;
import static org.firstinspires.ftc.teamcode.resources.hardware.relicRotationalServo;

/**
 * Created by dansm on 3/22/2018.
 */

public class RelicRecoverer extends Subsystem{
    DcMotorEx relicExtender;
    Servo relicRotationalServo;
    Servo relicExtensionServo;

    public enum RELIC_EXTENSION_SERVO_POSITION {
        INIT,
        UP,
        DOWN
    }

    public RELIC_EXTENSION_SERVO_POSITION relicExtensionServoPosition;
    public double relicExtenderPower;
    public double relicRotationalServoPosition;

    public void initHardware(OpMode opMode){
        relicExtender = opMode.hardwareMap.get(DcMotorEx.class, "relicMotor");
        relicRotationalServo = opMode.hardwareMap.get(Servo.class, "relicPivot");
        relicExtensionServo = opMode.hardwareMap.get(Servo.class, "relicLinear");

        relicExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicExtenderPower = 0;
    }


    public void update(){
        if(relicExtensionServoPosition == RELIC_EXTENSION_SERVO_POSITION.INIT || relicExtensionServoPosition == RELIC_EXTENSION_SERVO_POSITION.UP){
            relicExtensionServo.setPosition(1);
        }
        else if(relicExtensionServoPosition == RELIC_EXTENSION_SERVO_POSITION.DOWN){
            relicExtensionServo.setPosition(0);
        }

        relicExtender.setPower(relicExtenderPower);

        relicRotationalServo.setPosition(relicRotationalServoPosition);
    }
}
