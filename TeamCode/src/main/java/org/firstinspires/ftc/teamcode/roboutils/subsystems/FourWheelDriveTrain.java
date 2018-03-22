package org.firstinspires.ftc.teamcode.roboutils.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by dansm on 3/21/2018.
 */

public class FourWheelDriveTrain extends Subsystem{
    enum DRIVE_TRAIN_TYPE {TANK, MECANUM}

    public DRIVE_TRAIN_TYPE driveTrainType;

    DcMotorEx frontLeftDriveMotor;
    DcMotorEx frontRightDriveMotor;
    DcMotorEx backLeftDriveMotor;
    DcMotorEx backRightDriveMotor;

    public double frontLeftMotorSpeed;
    public double frontRightMotorSpeed;
    public double backLeftMotorSpeed;
    public double backRightMotorSpeed;

    public void initFourWheelDriveTrain(HardwareMap hardwareMap){
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "BackRight");


        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update(){
        frontLeftDriveMotor.setPower(frontLeftMotorSpeed);
        frontRightDriveMotor.setPower(frontRightMotorSpeed);
        backLeftDriveMotor.setPower(backLeftMotorSpeed);
        backRightDriveMotor.setPower(backRightMotorSpeed);
    }

}
