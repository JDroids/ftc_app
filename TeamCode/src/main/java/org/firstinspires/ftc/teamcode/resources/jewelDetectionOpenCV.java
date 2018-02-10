    package org.firstinspires.ftc.teamcode.resources;

import android.util.Log;
import android.util.Size;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.OpenCVPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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

    private double[] blueData;
    private double[] redData;

    private Mat b_hist = new Mat();
    private Mat r_hist = new Mat();
    private Mat croppedImage = new Mat();

    private Point p1 = new Point(336, 1462);
    private Point p4 = new Point(1020, 1914);

    private MatOfInt histSize = new MatOfInt(256);
    private MatOfInt channels = new MatOfInt(0);
    private Mat mask = new Mat();

    private Rect rectCrop;

    private MatOfFloat range = new MatOfFloat(0f, 256f);

    private int lastBlue;
    private int lastRed;

    private boolean uniform = true;
    private boolean accumulate = false;

    public JDColor jewelOnLeft = JDColor.NONE;

    private boolean firstTimeThroughLoop = true;

    private List<Mat> blue_plane = new ArrayList<Mat>();
    private List<Mat> red_plane = new ArrayList<Mat>();

    // This is called every camera frame.

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        blue_plane.clear();
        red_plane.clear();

        bgr_planes.clear();




        croppedImage = rgba; //Get rid of this

        /*
        //Cropping code; causes phone to black screen and close app (needs to be commented to be able to figure out where to crop using below code)
        if(firstTimeThroughLoop) {
            rectCrop = new Rect(((int) (rgba.width()*(1.0/9.0))), (int) (rgba.height() * (2.0/3.0)), (int) (rgba.width() * (8.0/9.0)), (int) (rgba.height() * (1.0/3.0)));
            firstTimeThroughLoop = false;
        }
        croppedImage = rgba.submat(rectCrop);
        Imgproc.resize(croppedImage, croppedImage, rgba.size());
        */


        //Used to determine where to crop (needs to be commented once the right cropping numbers are found)
        Point topLeftPoint = new Point((int) (rgba.width()*(1.0/9.0)), (int) (rgba.height() * (2.0/3.0)));
        Point bottomRightPoint = new Point(rgba.height(), rgba.width());
        Imgproc.rectangle(croppedImage, topLeftPoint, bottomRightPoint, new Scalar(73, 94, 49));


        Core.split(croppedImage, bgr_planes);

        blue_plane.add(bgr_planes.get(0));
        red_plane.add(bgr_planes.get(2));

        Imgproc.calcHist(blue_plane, channels, mask, b_hist, histSize, range);
        Imgproc.calcHist(red_plane, channels, mask, r_hist, histSize, range);

        b_hist.convertTo(b_hist, CvType.CV_64FC3);
        r_hist.convertTo(r_hist, CvType.CV_64FC3);

        //Blue/red data is null, b/r hist is not null
        blueData = new double[b_hist.width() - 1];
        redData = new double[r_hist.width() - 1];

        blueData = b_hist.get(b_hist.width() - 1, 0);
        redData = r_hist.get(r_hist.width() - 1, 0);



        lastBlue = (int) blueData[blueData.length - 1];
        lastRed = (int) redData[redData.length - 1];

        if(b_hist == null){
            Log.d("NullWatch", "b_hist is null");
        }
        if(blueData == null){
            Log.d("NullWatch", "blueData is null");
        }

        Log.d("Last Blue", Integer.toString(lastBlue));
        Log.d("Last Red", Integer.toString(lastRed));

        //Maybe if the ranges are within ~1000 of each other call it none?

        if(lastRed > lastBlue){
            Log.d("JewelColor", "Red");
            jewelOnLeft = JDColor.RED;
        }
        else if(lastBlue > lastRed){
            Log.d("JewelColor", "Blue");
            jewelOnLeft = JDColor.BLUE;
        }
        else{
            Log.d("JewelColor", "None");
            jewelOnLeft = JDColor.NONE;
        }

        return croppedImage;

    }
}