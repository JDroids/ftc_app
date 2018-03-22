package org.firstinspires.ftc.teamcode.roboutils.commands.miscellaneous;

import static org.firstinspires.ftc.teamcode.resources.constants.MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS;

/**
 * Created by dansm on 3/21/2018.
 */

public class InputScaling {
    static public double scaleInputFixedSpeed(double dVal){
        int sign;

        if (dVal <= -MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS || dVal >= MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS) {
            sign = (int) (dVal / Math.abs(dVal));
        } else {
            sign = 1;
            return 0.0;
        }

        double result = Math.abs(dVal);

        if (result < 0.3) {
            result = 0.25;
        } else if (result < 0.5) {
            result = 0.4;
        } else if (result < 0.7) {
            result = 0.6;
        } else {
            result = 0.75;
        }
        return result * sign;
    }

    static public double scaleInput(double dVal){
        double result = Math.pow(dVal, 3);
        if (result > 0.7) {
            result = 0.7;
        } else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

    static public double scaleInputPowerOf2(double dVal) {
        double sign = dVal / Math.abs(dVal);
        double result = (Math.pow(dVal, 2)) * sign;

        if (result > 0.7) {
            result = 0.7;
        } else if (result < -0.7) {
            result = -0.7;
        }
        return result;
    }

}
