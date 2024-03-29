package org.firstinspires.ftc.teamcode.drive.PurePursuitTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PurePursuit.AngledPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.robotClasses.RobotClass;
import org.opencv.core.Point;

import java.util.ArrayList;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitGlobals.*;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.followCurve;

@Autonomous
public class PurePursuitOpMode extends OpMode {


    // mecanum drivebase stolen from roadrunner
    private SampleMecanumDriveBase driveRR;
    // Pi Rho Robot class
    RobotClass robot = new RobotClass();

    private opModeStates driveState = opModeStates.NEXTPOINT;
    private int CURRENT_POINT_INDEX = 0;
    private ArrayList<AngledPoint> points = new ArrayList<>();
    @Override
    public void init() {
        robot.init(hardwareMap);
        initRoadRunner();
        points.add(new AngledPoint(0,0,toRadians(90)));
        points.add(new AngledPoint(60,-10,toRadians(90)));
        points.add(new AngledPoint(60,-60,toRadians(90)));
        points.add(new AngledPoint(0,-60,toRadians(180)));
        points.add(new AngledPoint(20,0,toRadians(90)));


    }
    @Override
    public void loop() {
        driveBaseLoop();

    }

    /**
     * Functionality that needs to loop in every single Pure Pursuit iterative opmode.
     */
    public void driveBaseLoop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(points.get(CURRENT_POINT_INDEX).x,
                points.get(CURRENT_POINT_INDEX).y, 1,0.4,10,
                1,1));


        switch (driveState) {
            case FOLLOW_PATH:
                posTurnToTarget = (!(points.get(CURRENT_POINT_INDEX).y < points.get(CURRENT_POINT_INDEX - 1).y)
                        || !(points.get(CURRENT_POINT_INDEX).x > points.get(CURRENT_POINT_INDEX - 1).x))
                        && (!(points.get(CURRENT_POINT_INDEX).y > points.get(CURRENT_POINT_INDEX - 1).y)
                        || !(points.get(CURRENT_POINT_INDEX).x < points.get(CURRENT_POINT_INDEX - 1).x));

                CurvePoint currentPoint = followCurve(allPoints, -Math.toRadians(180));
                // if the distance to the point is less than the threshold then move to the nextPoint

                if (worldDistanceToTarget < DISTANCE_THRESHOLD) {
                    driveState = opModeStates.NEXTPOINT;
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
                    startPath = new Point(points.get(CURRENT_POINT_INDEX).x,points.get(CURRENT_POINT_INDEX).y);
                    targetAngle = points.get(CURRENT_POINT_INDEX).angle;
                    driveState = opModeStates.FOLLOW_PATH;

                } else {
                    driveState = opModeStates.STOP;
                }

                break;
        }

        moveRobotPP();
        telemetry.addData("targetAngle: ",targetAngle);
        outputTelemetry();



    }

    /**
     * Initializes roadrunner localization and starting pose
     */
    public void initRoadRunner() {
        // init Mecanum drive base object so we can get position stuff
        driveRR = new SampleMecanumDriveREV(hardwareMap);
        driveRR.setPoseEstimate(new Pose2d(0,0,0));
    }

    /**
     * Method updates the position of the robot for purepursuit to function
     *
     * In order for purepursuit to function this method must be called in the loop method.
     */
    private void updatePose() {
        driveRR.update();
        worldXPosition = driveRR.getPoseEstimate().getX();
        // i have no idea why, this took me forever to figure out but the roadrunner y axis is opposite of the gluten free purepursuit axis
        worldYPosition = driveRR.getPoseEstimate().getY();
        // peter likes things at 90 degrees lol
        worldAngle_rad = driveRR.getPoseEstimate().getHeading() - Math.toRadians(180);
        // get robots raw IMU angle skipping roadruner
        robotRawAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    /**
     * Method moves the robot given powers created by pure pursuit
     */
    private void moveRobotPP() {
        // mecanum wheel powers
        double frontLeftPower = movement_x - movement_y - movement_turn;
        double frontRightPower = movement_x + movement_y + movement_turn;
        double backLeftPower = movement_x + movement_y - movement_turn;
        double backRightPower = movement_x - movement_y + movement_turn;

        // set motor powers
        driveRR.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        updatePose();
    }

    /**
     * Outputs basic telemetry of some basic global variables
     */
    private void outputTelemetry() {
        telemetry.addData("X Power: ",movement_x);
        telemetry.addData("Y Power: ", movement_y);
        telemetry.addData("Turn Power: ",movement_turn);
        telemetry.addData("______________","");
        telemetry.addData("X Pose: ",worldXPosition);
        telemetry.addData("Y Pose: ",worldYPosition);
        telemetry.addData("Heading: ",worldAngle_rad);
        telemetry.addData("Heading Degrees: ",toDegrees(worldAngle_rad));
        telemetry.addData("RAW Heading: ",robotRawAngle);
        telemetry.addData("Unadjusted Heading DEG: ",toDegrees(robotRawAngle));
        telemetry.update();
    }

    enum opModeStates {
        FOLLOW_PATH,
        STOP,
        NEXTPOINT, // due to a weird glitch where we can only follow one point at a time, this state will change the value in the list to move on
        NONE

    }

}
