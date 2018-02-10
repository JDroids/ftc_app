package org.firstinspires.ftc.teamcode.opmodes.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.resources.external.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.resources.jewelDetectionOpenCV;


/**
 * Created by dansm on 2/10/2018.
 */

@Autonomous(name="VisionTest")
public class visionTest extends LinearOpMode{
    @Override

    public void runOpMode() {
        jewelDetectionOpenCV jewelVision = new jewelDetectionOpenCV();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        jewelVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);

        ElapsedTime mRuntime = new ElapsedTime();


        mRuntime.reset();

        jewelVision.enable();
        while (!isStarted()) {
            telemetry.addData("Time Elapsed", mRuntime.milliseconds());
            telemetry.addData("Jewel On Left", jewelVision.jewelOnLeft);
            telemetry.update();
        }

        jewelVision.disable();

        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        ClosableVuforiaLocalizer vuforia;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        parameters.vuforiaLicenseKey = " AZcIMlr/////AAAAGe1W/L9P20hXupxJsIH5bIMDl46JPwjrX2kI+L6+tigIG9bhthzvrEWVBni6g4Jkvs76N/hIT0bFun78pnNDqkG3ZP24XLj45VHA2rYKp8UDww/vfy8xrtvHxedihdX1A2vMWg8Ub8tLjBMgEAqcAYYUMwPRQfI61KQmXvAJBV79XtQughxCh/fbrtoux6WV6HHs8OydP7kPUaUU3f0z5ZOF/TUvcqFFotqnLg/KwXMxxrouRyDGCIbpbP7cYabiR7ShIGvrYoRKtbpwxS3WLSjjTd7ynvoidYipWZ60e6t+wUCzdXahS8g0veYuTQ+vwBqljhtLUWnCUjbJh2jocjxV9kLGgqlPFCmLHZyurYkX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = new ClosableVuforiaLocalizer(parameters);

        while(opModeIsActive()) {
            telemetry.addData("Vumark", vuMark);
            telemetry.update();
        }
    }

}
