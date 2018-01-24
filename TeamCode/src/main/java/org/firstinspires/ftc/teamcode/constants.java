
package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Created by dansm on 12/11/2017.
 */

//A file to hold all constants

public class constants{
    static final double STRAFING_LIMIT = 0.1;
    static final double[] BOTTOM_SERVO_GRABBER_INIT_POSITION = new double[]{0.65, 0.36};
    static final double[] BOTTOM_SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 1.0};
    static final double[] BOTTOM_SERVO_GRABBER_OPEN_POSITION = new double[]{0.3, 0.6};
    static final double[] BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.5, 0.5};

    static final double[] TOP_SERVO_GRABBER_INIT_POSITION = new double[]{0.85, 0.2};
    static final double[] TOP_SERVO_GRABBER_CLOSE_POSITION = new double[]{0.4, 0.7};
    static final double[] TOP_SERVO_GRABBER_OPEN_POSITION = new double[]{0.5, 0.6};
    static final double[] TOP_SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.7, 0.4};

    static final double JEWEL_KNOCKER_INIT_POSITION = 0;
    static final double JEWEL_ARM_INIT_POSITION = 0.9;

    static final int FIRST_LIFT = 1;
    static final int SECOND_LIFT = 2;
    static final int UP = 1;
    static final int DOWN = -1;

    public enum JDColor {NONE, RED, BLUE};
    public enum FIELD_SIDE {RECOVERY_SIDE, JUDGE_SIDE};

    static final int BOTH_GRABBERS = 0;
    static final int BOTTOM_GRABBER = 1;
    static final int TOP_GRABBER = 2;

    static final int RED = 0;
    static final int BLUE = 1;

    static final double MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS = 0.1;
    static final double MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS = 3500;

    static final boolean TELEOP = true;
    static final boolean AUTONOMOUS = false;

    static final double GLYPH_LIFT_AUTO_SPEED = 0.7;

    static final int FRONT_FACING_CAMERA = 1;
    static final int BACK_CAMERA = 0;

    static final double LIFT_TWITCH_THRESHOLD = 0.15;

    static final Scalar LOWER_BLUE = new Scalar(90, 128, 30);
    static final Scalar UPPER_BLUE = new Scalar(170, 255, 255);

}
