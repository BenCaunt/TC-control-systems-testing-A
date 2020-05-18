package org.firstinspires.ftc.teamcode.PurePursuit;

public class Robot {

    // position of the robot
    public static double worldYPosition = 0;
    public static double worldXPosition = 0;
    public static double worldAngle_rad = 0;


    // remaining distance as calculated by RobotMovement.goToPosition
    public static double worldDistanceToTarget = 100;

    public static double globalAbsoluteAngleAlias = 0;
    public static double getGlobalRelativeAngleAlias = 0;
    // motor powers of the robot
    public static double movement_x = 0;
    public static double movement_y = 0;
    public static double movement_turn = 0;

    // direction that the robot turns which is based off if the next y value is greater or smaller than the previous
    public static boolean posTurnToTarget = true;

}
