package org.firstinspires.ftc.teamcode.opmodes.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.resources.external.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.resources.jewelDetectionOpenCV;

import static org.firstinspires.ftc.teamcode.resources.functions.getVumark;

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

        getVumark(this, hardwareMap);

    }

}
