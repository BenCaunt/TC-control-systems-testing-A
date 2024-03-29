package org.firstinspires.ftc.teamcode.drive.templates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.robotClasses.RobotClass;


// TODO: implement thing where we have a global angle and are constantly trying to turn to that ange
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


    /**
     *  GLOBALANGLE is important as it is constantly being updated throughout the OpMode.  The robot does its best to always be facing this angle
     *  This angle can infact be changed
     */
    public double GLOBALANGLE = 0;
    // PID coefficients for the PID controller
    // if using different motors than what comes default on the GoBilda Strafer as of 4/18/2020 this probably needs to be tuned
    private static double driveKp = 0.0008;
    private static double driveKi = 0.0001;
    private static double driveKd = 0;
    // PID constant for angle adjustment
    private static double adjustKp = 0;
    private static double adjustKi = 0.0;
    private static double adjustKd = 0.00;
    // error
    double angleError = 0;
    double I_angleError = 0;
    double D_angleError = 0;
    double previousAngleError = 0;



    public static double turnKp = 0.017;
    public static double turnKi = 0.0;
    public static double turnKd = 0;
    double turnError = 0;
    double I_turnError = 0;
    double D_turnError = 0;
    double previousTurnError = 0;


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

    /**
     * Method is called in runOpMode() before waitForStart() to setup all the motor and robot stuff
     */
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

    /**
     * Drive the robot a specific number of inches using encoders, imu, and a little PID magic
     * @param inches is the number of inches the robot should drive
     */
    public void encoderDriveIMU(double inches) {

        // angle that gets updated through the loop
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double robotAngle = 0;
        double speedL = 0;
        double speedR = 0;
        double speedAdjust = 0;
        // the value subtracted from the motor speed changes over time to create a speed ramp
        double ramp = 1;

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
                // drive loop duration
                double loopStartTime = runtime.milliseconds();
                // update robot angle
                robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

                // recalculate angle error
                angleError = startAngle - robotAngle;
                I_angleError = angleError * driveLoopIterationTime;
                D_angleError = ((angleError - previousAngleError) / driveLoopIterationTime);
                previousAngleError = angleError;
                // use pid to calculate the motor power adjustments
                speedAdjust = ((angleError * adjustKp) + (I_angleError * adjustKi) + (D_angleError * adjustKd));


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
                speedL = MotorPowerClipExtended(((errorL * driveKp) + (I_errorL * driveKi) + (D_errorL * driveKd))) - speedAdjust;
                /*
                *
                *  PID FOR RIGHT MOTORS
                *
                 */
                errorR = ((newRightBackTarget + newRightFrontTarget)/2) - (robot.BackRight.getCurrentPosition()+robot.FrontRight.getCurrentPosition())/2;
                I_errorR = (errorR * driveLoopIterationTime);
                D_errorL = ((errorR-previousErrorR)/driveLoopIterationTime);
                previousErrorR = errorR;
                speedR = MotorPowerClipExtended(((errorR * driveKp) + (I_errorR * driveKi) + (D_errorR * driveKd))) + speedAdjust;

                // we then clip the ramp value
                ramp = MotorPowerClip(ramp);

                // in order to smooth the robots movement we need to introduce ramping to the motors power
                // we calculate the new speed by taking the absolute speed and subtracting the ramp amount from it
                speedL = Math.abs(MotorPowerClipExtended(speedL) - ramp);
                speedR = Math.abs(MotorPowerClipExtended(speedR) - ramp);


                // decrement the clip value so it like ramps
                ramp -= 0.09;
                if (ramp < 0) {
                    ramp = 0;
                }

                // UPDATE MOTOR POWER
                robot.BackRight.setPower(Math.abs(speedR));
                robot.BackLeft.setPower(Math.abs(speedL));
                robot.FrontLeft.setPower(Math.abs(speedL));
                robot.FrontRight.setPower(Math.abs(speedR));
                // wait for the PID loop
                while ((runtime.milliseconds() < loopStartTime + driveLoopIterationTime) && opModeIsActive()) {

                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.BackLeft.getCurrentPosition(),
                        robot.BackRight.getCurrentPosition());
                telemetry.addData("left power: ",speedL);
                telemetry.addData("right power", speedR);
                telemetry.addData("left error: ",errorL);
                telemetry.addData("right error: ",errorR);

                telemetry.addData("Angle Error: ", angleError);
                telemetry.addData("Current angle",robotAngle);
                telemetry.addData("GLOBALANGLE: ",GLOBALANGLE);
                telemetry.update();

            }

            // Stop all motion;
            STOPDT();

            // Turn off RUN_TO_POSITION
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    /**
     * Turn TO a specific angle
     * @param angle is te angle that the robot shall turn towards
     */
    public void turnToAngle(double angle) {
        // init start time
        double loopStartTime;
        // update global angle to this new angle
        GLOBALANGLE = angle;
        /**
         * SUPER JANKY MATH ALERT (WHOO!)
         * So basically kids this weird set of if statements forces the angle in between 180 and -180
         * because thats what the IMU likes
         * I think
         * I hate everything
         * Ben Caunt - 4/27/2020 at 2:25 EST while fighting through the COVID-19 Pandemic
         * If any future members of 8300 or whatever team I end up on sees this please let me know lol
         * my insta should still be @BenCaunt1232 lol
         * so yeah, HMU if you see this
         * Then again ill  be 18+ so that may be kinda sus
         */
        if (GLOBALANGLE > 180) {
            GLOBALANGLE = 180 - GLOBALANGLE;
        } else if (GLOBALANGLE < -180) {
            GLOBALANGLE = 360 - GLOBALANGLE;
        }
        double motorPower;
        double robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        turnError = GLOBALANGLE - robotAngle;
        previousTurnError = turnError;
        double targetThresh = 0.8;
        while ((turnError > targetThresh || turnError < -targetThresh) && opModeIsActive()) {
            // time at the start of the loop
            loopStartTime = runtime.milliseconds();
            // calculate PID Error
            turnError = GLOBALANGLE - robotAngle;
            // calculate integral error
            I_turnError = turnError * driveLoopIterationTime;
            // Derivative error
            D_turnError = ((turnError - previousAngleError) / driveLoopIterationTime);
            // store previous error
            previousTurnError = turnError;
            // use pid to calculate the motor power adjustments
            motorPower = (turnError * turnKp) + (I_turnError * turnKi) + (D_turnError * turnKd);


            robot.BackLeft.setPower(-motorPower);
            robot.BackRight.setPower(motorPower);
            robot.FrontLeft.setPower(-motorPower);
            robot.FrontRight.setPower(motorPower);
            robotAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;


            telemetry.addData("Turn Error: ",turnError);
            telemetry.addData("Angle",robotAngle);
            telemetry.addData("Target",GLOBALANGLE);
            telemetry.addData("Power: ",motorPower);
            telemetry.update();

            // wait until the PID time thing says so before looping
            while (runtime.milliseconds() < loopStartTime + driveLoopIterationTime) {

            }

        }
        STOPDT();

    }

    /**
     * Stops the DriveTrain
     */
    public void STOPDT() {
        robot.BackRight.setPower(0);
        robot.BackLeft.setPower(0);
        robot.FrontLeft.setPower(0);
        robot.FrontRight.setPower(0);
    }


    /**
     * Takes in a motor power for RUN_TO_POSITION and clips it between 0 and 1
     * @param power is the motor power to be clipped
     * @return is the clipped value of the motor power
     */
    public double MotorPowerClip(double power) {
        if (power < 0) {
            return 0;
        } else if (power > 1) {
            return 1;
        } else {
            return power;
        }
    }

    /**
     * Takes in a motor power for RUN_USING_ENCODERS and clips it between -1 and 1
     * @param power
     * @return
     */
    public double MotorPowerClipExtended(double power) {
        if (power > 1) {
            return 1;
        } else if (power < -1) {
            return -1;
        } else {
            return power;
        }
    }
}

