package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

import java.util.ArrayList;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class MathFunctions {
    /**
     * Static method that ensures angle doesnt need more than one rotation
     *
     * Range is -180 to 180 degrees
     *
     * @param angle is a double that is the angle that we need to validate
     * @return angle is the new angle that is between the range
     */
    public static double AngleWrap(double angle) {
        // while the angle is less than -180 degrees
        while (angle < -Math.PI) {
            // add one full turn
            angle += 2 * Math.PI;
        }
        // while the angle is greater than 180 degrees
        while (angle > Math.PI) {
            // subtract a full turn
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Calculates the Line circle intersections needed for the pure pursuit algorithm
     * @param circleCenter
     * @param radius
     * @param linePoint1
     * @param linePoint2
     * @return arrayList of points that we follow
     */
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {

        // if the line is almost vertical or is extremely close just assume its some arbitrary close number
        // this is to prevent having an infinite slope and getting some scary errors
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        // calculate the slope of the first line
        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);
        // square the line slope
        double quadraticA = 1.0 + pow(m1,2);

        // math simplifies nicely if we define circleX/circleY as 0
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;


        // define quadratic b term
        double quadraticB = (2.0 * m1 * y1) - (2.0 + pow(m1,2) + x1);

        // define quadratic c term
        double quadraticC = ((pow(m1,2) * pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1,2) - pow(radius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        // super jank code where basically we calculate the quadratic formula and if we get an error we then know we need to use the negative version
        // because the quadratic has a plus or minus
        // and you can get  a divide by 0 error

        // this will fail and be caught whenever there is no line intersection
        // "then we got a problem -- peter 11115"

        try {
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // undo the offset to make the math good
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            // use a bounding box equation to make sure we are only using the actual line,  the equation assumes an infinitely long line so we make sure that we only use within the real line
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.y > linePoint2.y ? linePoint1.y : linePoint2.y;


            // make sure it is in the bounding box, if so we have found an intersection!!! wooo!!!
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        } catch (Exception e) {
            // if we are here ethe there are no roots to the circle so dont to anything
        }
        return allPoints;

    }
}
