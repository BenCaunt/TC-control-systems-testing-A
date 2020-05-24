package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

import static java.lang.Math.toDegrees;

/**
 * Class is a wrapper for an OpenCV point that includes an arbitrary angle set by the user to follow
 * This is so the user can
 */
public class AngledPoint {
    public double x;
    public double y;
    public double angle;

    public AngledPoint(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double angleToDegrees() {
        return toDegrees(angle);
    }

    public Point toOpenCVpoint() {
        return new Point(x,y);
    }

}
