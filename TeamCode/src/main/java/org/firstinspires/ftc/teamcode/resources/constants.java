
package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Created by dansm on 12/11/2017.
 */

//A file to hold all constants

public class constants{
    //{Left servo pos, right servo pos}
    static public final double STRAFING_LIMIT = 0.1;
    static public final double[] BOTTOM_SERVO_GRABBER_INIT_POSITION = new double[]{0.65, 0.36};
    static public final double[] BOTTOM_SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 1.0};
    static public final double[] BOTTOM_SERVO_GRABBER_OPEN_POSITION = new double[]{0.3, 0.6};
    static public final double[] BOTTOM_SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.5, 0.5};

    static public final double[] TOP_SERVO_GRABBER_INIT_POSITION = new double[]{1.0, 0.35};
    static public final double[] TOP_SERVO_GRABBER_CLOSE_POSITION = new double[]{0.0, 0.9};
    static public final double[] TOP_SERVO_GRABBER_OPEN_POSITION = new double[]{0.3, 0.7};
    static public final double[] TOP_SERVO_GRABBER_WIDE_OPEN_POSITION = new double[]{0.4, 0.6};

    static public final double JEWEL_KNOCKER_INIT_POSITION = 0;
    static public final double JEWEL_ARM_INIT_POSITION = 0.9;

    static public final int FIRST_LIFT = 1;
    static public final int SECOND_LIFT = 2;
    static public final int UP = 1;
    static public final int DOWN = -1;

    public enum JDColor {NONE, RED, BLUE};
    public enum FIELD_SIDE {RECOVERY_SIDE, JUDGE_SIDE};

    public enum GRABBERS {BOTH_GRABBERS, BOTTOM_GRABBER, TOP_GRABBER}

    static public final int RED = 0;
    static public final int BLUE = 1;

    static public final double MAX_NUMBER_WITHIN_RANGE_OF_TWITCHINESS = 0.1;
    static public final double MAX_RUNTIME_TO_CRYPTOWALL_MILLISECONDS = 3500;

    static public final boolean TELEOP = true;
    static public final boolean AUTONOMOUS = false;

    static public final double GLYPH_LIFT_AUTO_SPEED = 0.7;

    static public final int FRONT_FACING_CAMERA = 1;
    static public final int BACK_CAMERA = 0;

    static public final double LIFT_TWITCH_THRESHOLD = 0.15;

    static public final Scalar LOWER_BLUE = new Scalar(90, 128, 30);
    static public final Scalar UPPER_BLUE = new Scalar(170, 255, 255);

    static public final Scalar DARK_LOWER_RED = new Scalar(0, 113, 140);
    static public final Scalar DARK_UPPER_RED = new Scalar(11, 255, 255);

    static public final Scalar BRIGHT_LOWER_RED = new Scalar(166, 113, 140);
    static public final Scalar BRIGHT_UPPER_RED = new Scalar(180, 255, 255);
    
    public enum DIRECTION {MOVING_TOWARDS_OBJECT, MOVING_AWAY_FROM_OBJECT};
}
