package org.firstinspires.ftc.teamcode.resources;

import android.util.Log;

import org.firstinspires.ftc.teamcode.resources.external.PidUdpReceiver;

/**
 * Created by dansm on 2/28/2018.
 */

public class PID {
    private double kp;
    private double ki;
    private double kd;

    private double kpFromUdp;
    private double kiFromUdp;
    private double kdFromUdp;

    private double now;
    private double lastTime = 0;
    private double timeChange;

    private double previousError;

    private double error;
    private double errSum = 0;
    private double dErr = 0;

    private double output;

    public PidUdpReceiver pidUdpReceiver;

    public void setCoeffecients(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    public double calculateOutput(double target, double currentValue, boolean pidTuning) {
        if (pidTuning) {
            updateCoefficients();

            //How long since we calculated
            now = System.currentTimeMillis();
            timeChange = (double) (now - lastTime);

            //Computes error working variable
            error = target - currentValue;               //P
            errSum += (error * timeChange);              //I
            dErr = (error - previousError) / timeChange; //D

            output = (kpFromUdp * error) + (kiFromUdp * errSum) + (kdFromUdp * dErr);

            //Remember some stuff for next loop
            previousError = error;
            lastTime = System.currentTimeMillis();

            Log.d("JDPID", "CurrentVal: " + Double.toString(currentValue));
            Log.d("JDPID", "Output: " + Double.toString(output));


            return output;
        } else {
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

    public double calculateOutput(double target, double currentValue) {
        return calculateOutput(target, currentValue, false);
    }

    private boolean firstTime = true;

    public void updateCoefficients() {
        if (firstTime) {
            initializeUdpRecevier();
            firstTime = false;
        }
        kpFromUdp = pidUdpReceiver.getP();
        kiFromUdp = pidUdpReceiver.getI();
        kdFromUdp = pidUdpReceiver.getD();
    }


    public void initializeUdpRecevier() {
        if (firstTime) {
            pidUdpReceiver = new PidUdpReceiver();
            pidUdpReceiver.beginListening();
        }
    }
}
