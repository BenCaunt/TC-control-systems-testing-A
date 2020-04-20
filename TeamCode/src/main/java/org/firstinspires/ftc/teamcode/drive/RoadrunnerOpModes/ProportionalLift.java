package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class ProportionalLift extends LinearOpMode {
    private DcMotor lift;
    double liftPosition = 0;
    public static int targetPosition = 0;
    double error = 0;
    double I_error = 0;
    double D_error = 0;
    double previousError = 0;
    double motorPower = 0;
    public static double kp = 0.005;
    public static double ki = 0;
    public static double kd = 0.0005;
    public static long time = 10    ;

    @Override
    public void runOpMode() {
        lift = hardwareMap.dcMotor.get("armMotor");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            liftPosition = lift.getCurrentPosition();
            error = targetPosition - liftPosition;
            I_error = (error * time);
            D_error = ((error-previousError)/time);
            motorPower = (error * kp) + (I_error * ki) + (D_error * kd);
            lift.setPower(motorPower);
            telemetry.addData("position: ",liftPosition);
            telemetry.addData("Power: ",motorPower);
            telemetry.addData("Error: ",error);
            telemetry.update();
            sleep(time);
;        }
    }
}
