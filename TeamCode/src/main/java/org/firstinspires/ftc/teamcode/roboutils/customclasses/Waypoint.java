package org.firstinspires.ftc.teamcode.roboutils.customclasses;

/**
 * Created by dansm on 3/26/2018.
 */

public class Waypoint {
    public double x;
    public double y;
    public double heading;

    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Waypoint(double x, double y){
        this.x = x;
        this.y = y;
    }
}
