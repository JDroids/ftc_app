package org.firstinspires.ftc.teamcode.resources;

import android.util.Size;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.OpenCVPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.resources.constants.*;
import static org.firstinspires.ftc.teamcode.resources.functions.*;
import static org.firstinspires.ftc.teamcode.resources.hardware.*;


public class jewelDetectionOpenCV extends OpenCVPipeline {
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat thresholded_rgba = new Mat();

    private List<Mat> bgr_planes = new ArrayList<Mat>();

    double[] blueData;
    double[] redData;

    private Mat b_hist, r_hist;
    private Mat croppedImage = new Mat();

    private Point p1 = new Point(336, 1462);
    private Point p4 = new Point(1020, 1914);

    private MatOfInt histSize = new MatOfInt(256);
    private MatOfInt channels = new MatOfInt(0);
    private Mat mask = new Mat();

    private Rect rectCrop = new Rect(336, 1462, 685, 453);

    private MatOfFloat range[] = new MatOfFloat[]{new MatOfFloat(0), new MatOfFloat(256)};

    private int lastBlue;
    private int lastRed;

    private boolean uniform = true;
    private boolean accumulate = false;

    public JDColor jewelOnLeft = JDColor.NONE;

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        croppedImage = rgba.submat(rectCrop);

        Imgproc.resize(croppedImage, croppedImage, rgba.size());


        Core.split(croppedImage, bgr_planes);

        Imgproc.calcHist((List<Mat>) bgr_planes.get(0), channels, mask, b_hist, histSize, new MatOfFloat(256));
        Imgproc.calcHist((List<Mat>) bgr_planes.get(2), channels, mask, r_hist, histSize, new MatOfFloat(256));


        blueData = b_hist.get(b_hist.width() - 1, b_hist.height() - 1);
        redData = r_hist.get(r_hist.width() - 1, r_hist.height() - 1);

        lastBlue = (int) blueData[blueData.length - 1];
        lastRed = (int) redData[redData.length - 1];

        if(lastRed > lastBlue){
            jewelOnLeft = JDColor.RED;
        }
        else if(lastBlue > lastRed){
            jewelOnLeft = JDColor.RED;
        }
        else{
            jewelOnLeft = JDColor.NONE;
        }   


        return croppedImage;
    }
}