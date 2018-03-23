package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.resources.functions.moveWithMotionProfiling;
import static org.firstinspires.ftc.teamcode.resources.hardware.initHardwareMap;

/**
 * Created by dansm on 3/8/2018.
 */

@Disabled

@Autonomous(name = "Motion Profile Move")
public class motionProfilingTest extends LinearOpMode {
    public void runOpMode() {
        initHardwareMap(hardwareMap);

        waitForStart();
        /*
        try{
            BufferedReader br = new BufferedReader(new FileReader("/sdcard/move120CM.csv"));
            Log.d("JDFile", "File Read");

            String line = br.readLine();

            String[] lineValue = line.split(",");

            Log.d("JDFile", lineValue[0]);
        }
        catch (FileNotFoundException e){
            Log.d("JDFile", "File Not Found");
        }
        catch (IOException e){
            Log.d("JDFile", "IOException");
        }
        */

        moveWithMotionProfiling(30, "/sdcard/CSVFiles/30.0.csv", this);
    }
}
