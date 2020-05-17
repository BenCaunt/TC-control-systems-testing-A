package org.firstinspires.ftc.teamcode.drive.PurePursuitTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePursuit.Robot.*;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.ArrayList;

import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_turn;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_x;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.movement_y;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.Robot.worldYPosition;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement.gotToPosition;


@Autonomous
public class PurePursuitTest extends OpMode {
    // mecanum wheel powers
    double FrontLeftPower = 0;
    double FrontRightPower = 0;
    double BackLeftPower = 0;
    double BackRightPower = 0;
    // mecanum drivebase stolen from roadrunner
    SampleMecanumDriveBase driveRR;

    @Override
    public void init() {
        // init Mecanum drive base object so we can get position stuff
        driveRR = new SampleMecanumDriveREV(hardwareMap);
        driveRR.setPoseEstimate(new Pose2d(0,0.1,0));
        //driveRR.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
    }
    @Override
    public void loop() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(40,-20,0.3,0.5,10,1,1));
        allPoints.add(new CurvePoint(40,0,0.3,0.5,10,1,1));
        allPoints.add(new CurvePoint(0,0,0.3,0.5,10,1,1));
        CurvePoint currentPoint = followCurve(allPoints,-Math.toRadians(180));

        moveRobotPP();
        /*
        telemetry.addData("X Power: ",movement_x);
        telemetry.addData("Y Power: ", movement_y);
        telemetry.addData("______________","");
         */
        telemetry.addData("X Pose: ",worldXPosition);
        telemetry.addData("Y Pose: ",worldYPosition);
        telemetry.addData("Heading: ",worldAngle_rad);
        telemetry.addData("Heading Degrees: ",toDegrees(worldAngle_rad));


        telemetry.addData("Current x point: ",currentPoint.x);
        telemetry.addData("Current y point: ",currentPoint.y);
        telemetry.addData("Current point follow",currentPoint.followDistance);
        telemetry.update();


    }
    // updates position of the robot from roadrunner and stores it in the Robot class in the PurePursuit Lib

    /**
     * Method updates the position of the robot for purepursuit to function
     *
     * In order for purepursuit to function this method must be called in the loop method.
     */
    public void updatePurePursuitPose() {
        driveRR.update();
        worldXPosition = driveRR.getPoseEstimate().getX();
        // i have no idea why, this took me forever to figure out but the roadrunner y axis is opposite of the gluten free purepursuit axis
        worldYPosition = driveRR.getPoseEstimate().getY();
        // peter likes things at 90 degrees lol
        worldAngle_rad = driveRR.getPoseEstimate().getHeading() + Math.toRadians(90);
    }

    /**
     * Method moves the robot and updates position in the loop method
     *
     *
     */
    public void moveRobotPP() {
        FrontLeftPower = movement_x - movement_y + movement_turn;
        FrontRightPower = movement_x + movement_y - movement_turn;
        BackLeftPower = movement_x + movement_y + movement_turn;
        BackRightPower = movement_x - movement_y - movement_turn;

        // set motor powers
        driveRR.setMotorPowers(FrontLeftPower,BackLeftPower,BackRightPower,FrontRightPower);
        // update robot pose
        updatePurePursuitPose();
    }


}
