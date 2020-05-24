package org.firstinspires.ftc.teamcode.PurePursuit;

import org.opencv.core.Point;

public class PurePursuitGlobals {

    // position of the robot
    public static double worldYPosition = 0;
    public static double worldXPosition = 0;
    public static double worldAngle_rad = 0;


    // remaining distance as calculated by RobotMovement.goToPosition
    public static double worldDistanceToTarget = 100;

    public static double globalAbsoluteAngleAlias = 0;
    public static double getGlobalRelativeAngleAlias = 0;
    public static double relativeAngleToStart = 0;
    // motor powers of the robot
    public static double movement_x = 0;
    public static double movement_y = 0;
    public static double movement_turn = 0;
    // direction that the robot turns which is based off if the next y value is greater or smaller than the previous
    public static boolean posTurnToTarget = true;
    // minimum distance before switching to next point
    public static double DISTANCE_THRESHOLD = 2;

    // angle from an AngledPoint that the robot needs to face as commanded by an AngledPoint
    public static double targetAngle = 0;
    // ANGLE STRAIGHT FROM IMU, NOT ROADRUNNER
    public static double robotRawAngle = 0;
    // as part of our algorithm, we need to remember the start point between each path.
    public static Point startPath = new Point(0,0);

}
