package org.firstinspires.ftc.teamcode.resources;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
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

    private Mat b_hist, g_hist, r_hist;

    private Mat outputMat = new Mat();

    private MatOfInt histSize = new MatOfInt(256);
    private MatOfInt channels = new MatOfInt(0);
    private Mat mask = new Mat();

    private MatOfFloat range[] = new MatOfFloat[]{new MatOfFloat(0), new MatOfFloat(256)};


    private boolean uniform = true;
    private boolean accumulate = false;

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        Core.split(rgba, bgr_planes);


        Imgproc.calcHist((List<Mat>) bgr_planes.get(0), channels, mask, b_hist, histSize, new MatOfFloat(255),  accumulate);
        //Imgproc.calcHist(bgr_planes, 1, 0, g_hist, 1, histSize, range, uniform, accumulate);
        //Imgproc.calcHist(bgr_planes, 1, 0, r_hist, 1, histSize, range, uniform, accumulate);

        outputMat = b_hist;

        return outputMat;
    }
}