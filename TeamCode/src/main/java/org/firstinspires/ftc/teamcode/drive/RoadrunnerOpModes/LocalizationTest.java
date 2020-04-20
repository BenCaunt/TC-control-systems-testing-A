package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private DcMotor armMotor;
    private Servo clawArm;
    private Servo autoGrab;
    private Servo claw;
    private Servo sideClaw;
    private Servo sideClawDeploy;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawArm = hardwareMap.servo.get("clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");
        sideClawDeploy = hardwareMap.servo.get("sideClawDeploy");
        sideClaw = hardwareMap.servo.get("sideClaw");

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            // put platform claw up
            if (gamepad1.y) {
                autoClawUp();
            }
            if (gamepad1.a) {
                autoClawDown();
            }

            // control arm with left stick on gamepad 2
            armMotor.setPower(-gamepad2.left_stick_y / 1);

            // flip claw out
            if (gamepad2.x) {
                armOut();
            }
            // flip claw in
            if (gamepad2.b) {
                armIn();
            }
            // grab stone
            if (gamepad2.a) {
                closeClaw();
            }

            // release stone
            if (gamepad2.y) {
                openClaw();
            }

            // platform grab down
            if (gamepad2.left_stick_button) {
                autoClawDown();
            }
            if (gamepad2.right_stick_button){
                autoClawUp();
            }

            // side claw stuff
            if (gamepad1.right_bumper) {
                sideClawIn();
            }
            if (gamepad1.left_bumper) {
                sideClawOut();
            }

            if (gamepad1.left_stick_button) {
                sideClawClose();
            }

            if (gamepad1.right_stick_button) {
                sideClawOpen();
            }


        }

    }

    public void armOut() {
        clawArm.setPosition(0.7);
    }

    public void sideClawIn() {
        sideClawDeploy.setPosition(0.3);

    }
    public void sideClawOut() {
        sideClawDeploy.setPosition(1);

    }

    public void sideClawOpen() {
        sideClaw.setPosition(0.5);
    }

    public void sideClawClose() {
        sideClaw.setPosition(1);
    }

    public void armIn() {
        clawArm.setPosition(0.04);
    }

    public void autoClawDown() {
        autoGrab.setPosition(1);

    }
    public void autoClawUp() {
        autoGrab.setPosition(0.35);
    }

    private void closeClaw() {
        claw.setPosition(0.99);
    }




    private void openClaw() {
        claw.setPosition(0.65);
    }
}
