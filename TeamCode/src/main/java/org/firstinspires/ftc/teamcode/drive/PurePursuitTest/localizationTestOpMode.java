package org.firstinspires.ftc.teamcode.drive.PurePursuitTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous
public class localizationTestOpMode extends OpMode {
    SampleMecanumDriveBase driveRR;
    private ElapsedTime runtime = new ElapsedTime();
    private opModeStates state = opModeStates.DRIVE;
    @Override
    public void init() {
        driveRR = new SampleMecanumDriveREV(hardwareMap);
        telemetry.addData("Ready For Start: ","True");
        telemetry.update();

    }
    @Override
    public void loop() {



        switch (state) {
            case NONE:
                break;
            case DRIVE:
                // strafe right
                driveRR.setMotorPowers(1,-1,-1,1);
                state = opModeStates.NONE;
                break;
            case STOP:
                driveRR.setMotorPowers(0,0,0,0);
                state = opModeStates.NONE;
                break;
        }


        driveRR.update();

        telemetry.addData("X Pose: ",driveRR.getPoseEstimate().getX());
        telemetry.addData("Y Pose: ",driveRR.getPoseEstimate().getY());
        telemetry.addData("Heading: ",driveRR.getPoseEstimate().getHeading());
        telemetry.update();
    }
    public void delay(long time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() < startTime + time) {

        }

    }

    public enum opModeStates {
        NONE,
        DRIVE,
        STOP
    }
}
