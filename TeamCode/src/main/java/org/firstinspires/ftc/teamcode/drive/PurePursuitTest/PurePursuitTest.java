package org.firstinspires.ftc.teamcode.drive.PurePursuitTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Robot.*;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.opencv.core.Point;

import java.lang.reflect.Array;
import java.util.ArrayList;

import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.getGlobalRelativeAngleAlias;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.globalAbsoluteAngleAlias;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_turn;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_x;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_y;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.posTurnToTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldDistanceToTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.gotToPosition;


@Autonomous
public class PurePursuitTest extends OpMode {

    final double DISTANCE_THRESHOLD = 5;
    CurvePoint currentPoint;
    // mecanum wheel powers
    double FrontLeftPower = 0;
    double FrontRightPower = 0;
    double BackLeftPower = 0;
    double BackRightPower = 0;
    // mecanum drivebase stolen from roadrunner
    SampleMecanumDriveBase driveRR;
    // state object that powers our state machine
    opModeStates state = opModeStates.FOLLOW_PATH;

    // current point the robot is traveling to. WE START at one because we add 0,0 to the list but we dont want to travel there
    int CURRENT_POINT_INDEX = 1;
    // arraylist of points that the robot will travel in total.
    ArrayList<Point> points = new ArrayList<>();
    ArrayList<Double> pointPowers = new ArrayList<>();
    ArrayList<Double> turnPowers = new ArrayList<>();
    @Override
    public void init() {
        // init Mecanum drive base object so we can get position stuff
        driveRR = new SampleMecanumDriveREV(hardwareMap);
        driveRR.setPoseEstimate(new Pose2d(0,0.1,0));
        points.add(new Point(0,0));
        pointPowers.add(new Double(0.3));
        turnPowers.add(new Double(0.05));


        points.add(new Point(40,0));
        pointPowers.add(new Double(0.3));
        turnPowers.add(new Double(0.05));

        points.add(new Point(40,-40));
        pointPowers.add(new Double(0.3));
        turnPowers.add(new Double(0.05));

        points.add(new Point(0,-40));
        pointPowers.add(new Double(0.3));
        turnPowers.add(new Double(0.05));

        points.add(new Point(0,0));
        pointPowers.add(new Double(0.3));
        turnPowers.add(new Double(0.05));

    }
    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(points.get(CURRENT_POINT_INDEX).x,points.get(CURRENT_POINT_INDEX).y,1,0.2,10,1,1));


        switch (state) {
            case FOLLOW_PATH:
                if (points.get(CURRENT_POINT_INDEX).y < points.get(CURRENT_POINT_INDEX - 1).y) {
                    posTurnToTarget = false;
                } else {
                    posTurnToTarget = true;
                }
                currentPoint = followCurve(allPoints,-Math.toRadians(180));
                // if the distance to the point is less than the threshold then move to the nextPoint
                if (worldDistanceToTarget < DISTANCE_THRESHOLD) {
                    state = opModeStates.NEXTPOINT;
                }
                break;
            case STOP:
                movement_x = 0;
                movement_y = 0;
                movement_turn = 0;
                break;

            case NEXTPOINT:
                // if there is another point available move to that point, if not then STOP the robot.
                if (CURRENT_POINT_INDEX + 1 < points.size()) {
                    CURRENT_POINT_INDEX += 1;
                    state = opModeStates.FOLLOW_PATH;
                } else {
                    state = opModeStates.STOP;
                }

                // if the upcoming point has a smaller y value than the previous turn a specific way

                break;

        }
        // set motor powers according to pure pursuit / robot state.

        /*
        telemetry.addData("Robot state:",state);
        telemetry.addData("Current Point X:",currentPoint.x);
        telemetry.addData("Current point y:",currentPoint.y);
        telemetry.addData("Relative Angle: ",getGlobalRelativeAngleAlias);
        telemetry.addData("REL DEG: ", toDegrees(getGlobalRelativeAngleAlias));

         */
        /*

        // angle debug stuff dont delete its useful
        telemetry.addData("ABSOLUTE ANGLE: ",globalAbsoluteAngleAlias);
        telemetry.addData("ABS DEG: ",toDegrees(globalAbsoluteAngleAlias));
        telemetry.addData("robot angle: ",worldAngle_rad);
        telemetry.addData("DEG robot: ",toDegrees(worldAngle_rad));
        telemetry.addData("Turn Power: ",movement_turn);


         */
        moveRobotPP();
        outputTelemetry();





    }
    // updates position of the robot from roadrunner and stores it in the Robot class in the PurePursuit Lib

    /**
     * Method updates the position of the robot for purepursuit to function
     *
     * In order for purepursuit to function this method must be called in the loop method.
     */
    public void updatePose() {
        driveRR.update();
        worldXPosition = driveRR.getPoseEstimate().getX();
        // i have no idea why, this took me forever to figure out but the roadrunner y axis is opposite of the gluten free purepursuit axis
        worldYPosition = driveRR.getPoseEstimate().getY();
        // peter likes things at 90 degrees lol
        worldAngle_rad = driveRR.getPoseEstimate().getHeading() - Math.toRadians(180);
    }

    /**
     * Method moves the robot given powers created by pure pursuit
     */
    public void moveRobotPP() {
        FrontLeftPower = movement_x - movement_y - movement_turn;
        FrontRightPower = movement_x + movement_y + movement_turn;
        BackLeftPower = movement_x + movement_y - movement_turn;
        BackRightPower = movement_x - movement_y + movement_turn;

        // set motor powers
        driveRR.setMotorPowers(FrontLeftPower,BackLeftPower,BackRightPower,FrontRightPower);
        updatePose();
    }

    /**
     * Outputs basic telemetry of some basic global variables
     */
    public void outputTelemetry() {
        telemetry.addData("X Power: ",movement_x);
        telemetry.addData("Y Power: ", movement_y);
        telemetry.addData("Turn Power: ",movement_turn);
        telemetry.addData("______________","");
        telemetry.addData("X Pose: ",worldXPosition);
        telemetry.addData("Y Pose: ",worldYPosition);
        telemetry.addData("Heading: ",worldAngle_rad);
        telemetry.addData("Heading Degrees: ",toDegrees(worldAngle_rad));
        telemetry.update();
    }
    enum opModeStates {
        FOLLOW_PATH,
        STOP,
        NEXTPOINT, // due to a weird glitch where we can only follow one point at a time, this state will change the value in the list to move on
        NONE

    }

}
