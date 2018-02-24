package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;

/**
 * Created by dansm on 2/23/2018.
 */

@TeleOp(name="Linear Servo test")

public class linearServoTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        initHardwareMap(hardwareMap);

        waitForStart();

        initServos(true);

        while(opModeIsActive()){
            if(gamepad1.right_bumper){ //To open
                if (relicLinearServo.getPosition() < 0.9) {
                    relicLinearServo.setPosition(relicLinearServo.getPosition() + 0.01);
                }
            }
            else if(gamepad1.left_bumper){ //To close
                if(relicLinearServo.getPosition() > 0.3){
                    relicLinearServo.setPosition(relicLinearServo.getPosition() - 0.01);
                }
            }
            else if(gamepad1.a){
                relicLinearServo.setPosition(0.5);
            }
            else if(gamepad1.b){
                relicLinearServo.setPosition(0.8);
            }
        }

    }
}
