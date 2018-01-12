package org.firstinspires.ftc.teamcode;
  
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kevin on 12/17/17.
 */

 public class PotentialVuforia extends LinearOpMode {
   @Override
    public void runOpMode() throws InterruptedException {
       VuforiaLocalizer.Parameters para = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
       para.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
       para.vuforiaLicenseKey = "AZcIMlr/////AAAAGe1W/L9P20hXupxJsIH5bIMDl46JPwjrX2kI+L6+tigIG9bhthzvrEWVBni6g4Jkvs76N/hIT0bFun78pnNDqkG3ZP24XLj45VHA2rYKp8UDww/vfy8xrtvHxedihdX1A2vMWg8Ub8tLjBMgEAqcAYYUMwPRQfI61KQmXvAJBV79XtQughxCh/fbrtoux6WV6HHs8OydP7kPUaUU3f0z5ZOF/TUvcqFFotqnLg/KwXMxxrouRyDGCIbpbP7cYabiR7ShIGvrYoRKtbpwxS3WLSjjTd7ynvoidYipWZ60e6t+wUCzdXahS8g0veYuTQ+vwBqljhtLUWnCUjbJh2jocjxV9kLGgqlPFCmLHZyurYkX";
       para.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

       VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(para);
       Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

       VuforiaTrackables Trackie = vuforia.loadTrackablesFromAsset("FTC_2016-17");
       Trackie.get(0).setName("Wheels");
       Trackie.get(1).setName("Tools");
       Trackie.get(2).setName("Lego");
       Trackie.get(3).setName("Gears");

       waitForStart();

       Trackie.activate();

       while (opModeIsActive()){
           for(VuforiaTrackable track : Trackie);
                //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) .getListener()).getPose();
            }
        }
    }