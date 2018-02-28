package org.firstinspires.ftc.teamcode.resources;

import org.firstinspires.ftc.teamcode.resources.external.PidUdpReceiver;

/**
 * Created by dansm on 2/28/2018.
 */

public class PID {
    static public double kp;
    static public double ki;
    static public double kd;

    static public double kpFromUdp;
    static public double kiFromUdp;
    static public double kdFromUdp;

    static private double now;
    static private double lastTime = 0;
    static private double timeChange;

    static private double previousError;

    static private double error;
    static private double errSum = 0;
    static private double dErr = 0;

    static private double output;

    static private PidUdpReceiver pidUdpReceiver;

    public static double calculateOutput(double target, double currentValue, boolean pidTuning){
        if(pidTuning){
            //How long since we calculated
            now = System.currentTimeMillis();
            timeChange = (double) (now - lastTime);

            //Computes error working variable
            error = target - currentValue;
            errSum += (error * timeChange);
            dErr = (error - previousError) / timeChange;

            output = (kpFromUdp * error) + (kiFromUdp * errSum) + (kdFromUdp * dErr);

            //Remember some stuff for next loop
            previousError = error;
            lastTime = System.currentTimeMillis();

            return output;
        }
        else {
            //How long since we calculated
            now = System.currentTimeMillis();
            timeChange = (double) (now - lastTime);

            //Computes error working variable
            error = target - currentValue;
            errSum += (error * timeChange);
            dErr = (error - previousError) / timeChange;

            output = (kp * error) + (ki * errSum) + (kd * dErr);

            //Remember some stuff for next loop
            previousError = error;
            lastTime = System.currentTimeMillis();

            return output;
        }
    }

    public static double calculateOutput(double target, double currentValue){
        return calculateOutput(target, currentValue, false);
    }

    public static void updateCoefficients(){
        initializeUdpRecevier();

        kpFromUdp = pidUdpReceiver.getP();
        kiFromUdp = pidUdpReceiver.getI();
        kdFromUdp = pidUdpReceiver.getD();
    }

    static private boolean firstTime;

    public static void initializeUdpRecevier(){
        if(firstTime){
            pidUdpReceiver = new PidUdpReceiver();
            pidUdpReceiver.beginListening();

            firstTime = false;
        }
    }
}
