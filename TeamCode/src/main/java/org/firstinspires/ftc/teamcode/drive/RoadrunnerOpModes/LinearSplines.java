package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")

public class LinearSplines extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder().splineTo(new Pose2d(15,15,0),new LinearInterpolator(0,270))
                .build()
        );

        sleep(2000);

    }
}
