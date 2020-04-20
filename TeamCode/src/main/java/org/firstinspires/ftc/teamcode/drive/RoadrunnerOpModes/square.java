package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous
public class square extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        // init stuff
        waitForStart();
        double positionThing = 50;
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .lineTo(new Vector2d(positionThing,0))
                        .strafeTo(new Vector2d(positionThing,-positionThing))

                .build()
        );


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse()
                        .lineTo(new Vector2d(0,-positionThing))
                        .strafeTo(new Vector2d(0,0))
                .build()
        );
    }

}
