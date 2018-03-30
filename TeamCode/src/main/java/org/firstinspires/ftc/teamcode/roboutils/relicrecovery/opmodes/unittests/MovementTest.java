package org.firstinspires.ftc.teamcode.roboutils.relicrecovery.opmodes.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roboutils.customclasses.Waypoint;
import org.firstinspires.ftc.teamcode.roboutils.relicrecovery.subsystems.RelicRecoveryRobot;
import org.firstinspires.ftc.teamcode.roboutils.templates.CustomOpMode;

/**
 * Created by dansm on 3/24/2018.
 */

@TeleOp(name="MovementTest", group="Tests")

public class MovementTest extends CustomOpMode{
    @Override
    public void runOpMode(){
        robot = new RelicRecoveryRobot(this, new Waypoint(0, 0));

        robot.initHardware(this);

        waitForStart();

        robot.initServosForTeleop();

        robot.update();

        sleep(1000);

        while(opModeIsActive()) {
            if (gamepad1.right_bumper){
                robot.drive.setMotorPower(-0.3, -0.3, -0.3, -0.3);
            }
            else if(gamepad1.left_bumper){
                robot.drive.setMotorPower(0.3, 0.3, 0.3, 0.3);
            }
            else if(gamepad1.dpad_up){
                robot.drive.setMotorPower(-0.3, 0.3, -0.3, 0.3); //Move forwards
            }
            else if(gamepad1.dpad_down){
                robot.drive.setMotorPower(0.3, -0.3, 0.3, -0.3); //Move backwards
            }
            else if(gamepad1.dpad_left){
                robot.drive.setMotorPower(0.3, 0.3, -0.3, -0.3); //Move left
            }
            else if(gamepad1.dpad_right){
                robot.drive.setMotorPower(-0.3, -0.3, 0.3, 0.3); //Move right
            }
            else{
                robot.drive.stopDriveMotors();
            }

            robot.update();
        }

        robot.drive.moveAtPower(0);
        robot.update();
    }
}
