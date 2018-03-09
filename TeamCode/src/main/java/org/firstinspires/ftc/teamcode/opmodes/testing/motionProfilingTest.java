package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import static org.firstinspires.ftc.teamcode.resources.hardware.*;
import static org.firstinspires.ftc.teamcode.resources.functions.*;

/**
 * Created by dansm on 3/8/2018.
 */

@Autonomous(name = "Motion Profile Move")
public class motionProfilingTest extends LinearOpMode{
    public void runOpMode(){
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

        moveWithMotionProfiling(120, "/sdcard/move120CM.csv", this);
    }
}
