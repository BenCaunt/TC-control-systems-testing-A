package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")

public class SplineTest extends LinearOpMode {
    Trajectory trajectory1;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        trajectory1 = drive.trajectoryBuilder()
                .reverse()
                .lineTo(new Vector2d(30,0))

                .build();

        waitForStart();

        telemetry.addData("Math.toRadians(90):",Math.toRadians(180));
        telemetry.update();
        if (isStopRequested()) return;



        drive.turnSync(Math.toRadians(90));

        drive.followTrajectorySync(trajectory1);
        while (opModeIsActive()) {
            if (true) {
                drive.update();
            } else {
                break;
            }
        }

        
    }
}
