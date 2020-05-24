package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Point;

import java.util.ArrayList;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitGlobals.*;

public class RobotMovement {


    public static CurvePoint followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        // TODO: currently we only use the follow distance of the last point in the list, make it better
        // TODO: ^ potentially the closest point to the robot physically.

        CurvePoint followMe = getFollowPointPath(allPoints,new Point(worldXPosition,worldYPosition),allPoints.get(allPoints.size()-1).followDistance);

        gotToPosition(followMe.x, followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed);

        return followMe;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        // if there is no intersections, get the first point (?)
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        // since we are iterated from 0 to the last point order sets priority, the last point in the list is the most desired
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius,startLine.toPoint(),
                    endLine.toPoint());

            // if we have two intersections, we chose the one with the closet angle to the robots angle

            double closestAngle = 1000000;

            for (Point thisIntersection : intersections) {
                // absolute angle to world coordinate space
                double angle = Math.atan2(thisIntersection.y-worldYPosition,thisIntersection.x-worldXPosition);
                double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }

        return followMe;
    }

    public static void gotToPosition(double x, double y, double movement_speed, double preferredAngle, double turnSpeed) {

        // something i added to make it go the right way
        //y = -y;
        // calculate the distance from the robots current pose to the target pose
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        worldDistanceToTarget = distanceToTarget;
        // calculate the angle to the target given the assumption the robot is at exactly 0 degrees
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double absoluteAngleToStart = Math.atan2(startPath.y - worldYPosition, startPath.x - worldXPosition);
        globalAbsoluteAngleAlias = absoluteAngleToTarget;
        // calculate the true angle to the target
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - toRadians(90)));
        relativeAngleToStart = AngleWrap(absoluteAngleToStart - (worldAngle_rad - toRadians(90)));

        double NormalrelativeAngleToPoint = AngleWrap(absoluteAngleToTarget - ((worldAngle_rad + toRadians(270)) - toRadians(90)));

        // distance traveled on the x axis
        double relativeXToPoint = Math.cos(NormalrelativeAngleToPoint) * distanceToTarget;
        // distance traveled on the y axis
        double relativeYToPoint = Math.sin(NormalrelativeAngleToPoint) * distanceToTarget;

        // no matter the total magnitude of the vector we are traveling, the movement power is guaranteed to be between 0 and 1
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        // update the static variables in the PurePursuit robot class that contain the x and y robot powers
        movement_x = movementXPower * movement_speed;
        movement_y = movementYPower * movement_speed;


        double relativeTurnAngleStart = relativeAngleToStart + preferredAngle;
        getGlobalRelativeAngleAlias = relativeTurnAngleStart;

        movement_turn = setTurnToAngleSpeed(targetAngle);//Range.clip((worldAngle_rad) - targetAngle,-1,1) * turnSpeed;//Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;
        // reverse movement_turn if necessary.
        if (true) {

        } else {
            movement_turn = -movement_turn;
        }
        // if we are less than 10 centimeters (3.93701 inches) we dont want to turn anymore to increase stability.
        if (distanceToTarget < 3.93701) {
            movement_turn = 0;
        }



    }

    public static double setTurnToAngleSpeed(double angle) {
        // init start time
        // update global angle to this new angle
        /**
         * SUPER JANKY MATH ALERT (WHOO!)
         * So basically kids this weird set of if statements forces the angle in between 180 and -180
         * because thats what the IMU likes
         * I think
         * I hate everything
         * Ben Caunt - 4/27/2020 at 2:25 EST while fighting through the COVID-19 Pandemic
         * If any future members of 8300 or whatever team I end up on sees this please let me know lol
         * my insta should still be @BenCaunt1232 lol
         * so yeah, HMU if you see this
         * Then again ill  be 18+ so that may be kinda sus
         */
        if (angle > 180) {
            angle = 180 - angle;
        } else if (angle < -180) {
            angle = 360 - angle;
        }

        return (angle - robotRawAngle) * 0.4;


    }



}
