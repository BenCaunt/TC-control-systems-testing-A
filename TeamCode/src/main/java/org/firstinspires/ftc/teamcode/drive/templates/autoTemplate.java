package org.firstinspires.ftc.teamcode.drive.templates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.robotClasses.PIDcontroller;
import org.firstinspires.ftc.teamcode.drive.robotClasses.RobotClass;

@Config
@Autonomous
public class autoTemplate extends LinearOpMode {
    // fairly precise timer I hope
    private ElapsedTime runtime = new ElapsedTime();
    // Drive train information
    final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Neverest motor 28 ticks times the 40:1 gear reduction of the neverest 40
    final double     DRIVE_GEAR_REDUCTION    = 45/35 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // PID coefficients for the PID controller
    // if using different motors than what comes default on the GoBilda Strafer as of 4/18/2020 this probably needs to be tuned
    public static double driveKp = 0.0008;
    public static double driveKi = 0;
    public static double driveKd = 0;

    public double angleKp = 0;
    public double angleKi = 0;
    public double angleKd = 0;
    public static double turnKp = 0.001;

    double errorL = 0;
    double I_errorL = 0;
    double D_errorL = 0;
    double previousErrorL = 0;

    double errorR = 0;
    double I_errorR = 0;
    double D_errorR = 0;
    double previousErrorR = 0;

    // this is the length of each iteration of the PID loop, in this case, 50 milliseconds
    public long driveLoopIterationTime = 10;
    // instance of robot class
    RobotClass robot = new RobotClass();
    // instance of our PID controller for our Drivebase

    public void SetUp() {
        robot.init(hardwareMap);
        robot.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    @Override
    public void runOpMode() {
        SetUp();
        waitForStart();

    }
    /*
    * function drives left and right motor to desired number of ticks
    * @param leftTicks is the amount of ticks the DT will move

     */
    public void encoderDriveIMU(double inches) {
        // current angle of the robot on start of method call
        float startHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        double speedL = 0;
        double speedR = 0;


        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            /*
            BackRight
            BackLeft
            FrontLeft
            FrontRight
            */
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = robot.BackLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.BackRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.FrontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.FrontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.BackRight.setTargetPosition(newRightBackTarget);
            robot.BackLeft.setTargetPosition(newLeftBackTarget);
            robot.FrontLeft.setTargetPosition(newLeftFrontTarget);
            robot.FrontRight.setTargetPosition(newRightFrontTarget);
            // Turn On RUN_TO_POSITION

            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            robot.BackRight.setPower(Math.abs(speedR));
            robot.BackLeft.setPower(Math.abs(speedL));
            robot.FrontLeft.setPower(Math.abs(speedL));
            robot.FrontRight.setPower(Math.abs(speedR));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.BackRight.isBusy() && robot.BackLeft.isBusy() && robot.FrontRight.isBusy() && robot.FrontLeft.isBusy())) {
                /*
                *
                * PID FOR LEFT MOTORS
                *
                *
                 */
                errorL = ((newLeftBackTarget + newLeftFrontTarget)/2) - (robot.BackLeft.getCurrentPosition()+robot.FrontLeft.getCurrentPosition())/2;
                I_errorL = (errorL * driveLoopIterationTime);
                D_errorL = ((errorL-previousErrorL)/driveLoopIterationTime);
                previousErrorL = errorL;
                speedL = (errorL * driveKp) + (I_errorL * driveKi) + (D_errorL * driveKd);
                /*
                *
                *  PID FOR RIGHT MOTORS
                *
                 */
                errorR = ((newRightBackTarget + newRightFrontTarget)/2) - (robot.BackRight.getCurrentPosition()+robot.FrontRight.getCurrentPosition())/2;
                I_errorR = (errorR * time);
                D_errorL = ((errorR-previousErrorR)/driveLoopIterationTime);
                previousErrorR = errorR;
                speedR = (errorR * driveKp) + (I_errorR * driveKi) + (D_errorR * driveKd);


                // Save gyro's yaw angle
                robot.Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw angle", robot.Yaw_Angle);
                // the speed is the output of the PID controller of the average target and the average current position
                // amount the motor speed is reduced by to counteract change in angle
                double adjustAmount = 0.5;

                if (robot.Yaw_Angle < startHeading - 5) {
                    // Turn left by letting right slightly overpower
                    robot.BackRight.setPower(Math.abs(speedR));
                    robot.BackLeft.setPower(Math.abs(speedL) - adjustAmount);
                    robot.FrontLeft.setPower(Math.abs(speedL) - adjustAmount);
                    robot.FrontRight.setPower(Math.abs(speedR));
                } else if (robot.Yaw_Angle > startHeading + 5) {
                    // Turn right by letting left slightly overpower
                    robot.BackRight.setPower(Math.abs(speedR) - adjustAmount);
                    robot.BackLeft.setPower(Math.abs(speedL));
                    robot.FrontLeft.setPower(Math.abs(speedL));
                    robot.FrontRight.setPower(Math.abs(speedR) - adjustAmount);
                } else {
                    // Continue straight
                    robot.BackRight.setPower(Math.abs(speedR));
                    robot.BackLeft.setPower(Math.abs(speedL));
                    robot.FrontLeft.setPower(Math.abs(speedL));
                    robot.FrontRight.setPower(Math.abs(speedR));
                }


                // wait for the PID loop
                sleep(driveLoopIterationTime);
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.BackLeft.getCurrentPosition(),
                        robot.BackRight.getCurrentPosition());
                telemetry.addData("left power: ",speedL);
                telemetry.addData("right power", speedR);
                telemetry.addData("left error: ",errorL);
                telemetry.addData("right error: ",errorR);
                telemetry.update();
            }

            // Stop all motion;

            STOPDT();

            // Turn off RUN_TO_POSITION
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void turnAngle(double angle) {

        double robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        double error = angle - robotAngle;
        double targetThresh = 0.8;
        while ((error > targetThresh || error < -targetThresh) && opModeIsActive()) {
            robot.BackLeft.setPower(-error * turnKp);
            robot.BackRight.setPower(error * turnKp);
            robot.FrontLeft.setPower(-error* turnKp);
            robot.FrontRight.setPower(error * turnKp);
            robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            error = angle - robotAngle;

            telemetry.addData("Turn Error: ",error);
            telemetry.addData("Angle",robotAngle);
            telemetry.addData("Target",angle);
            telemetry.update();

        }
        STOPDT();

    }
    public void STOPDT() {
        robot.BackRight.setPower(0);
        robot.BackLeft.setPower(0);
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
    }

}

