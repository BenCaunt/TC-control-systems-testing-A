package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.robotClasses.PIDcontroller;

@Config
@TeleOp
public class PIDcontrollerLiftExample extends LinearOpMode {
    private DcMotor lift;
    double liftPosition = 0;
    public static int targetPosition = 0;
    double motorPower = 0;
    public static double kp = 0.005;
    public static double ki = 0;
    public static double kd = 0.0005;
    public static long time = 10    ;
    public PIDcontroller PID;
    @Override
    public void runOpMode() {
        PID = new PIDcontroller(kp,ki,kd,time);
        lift = hardwareMap.dcMotor.get("armMotor");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            liftPosition = lift.getCurrentPosition();
            motorPower = PID.updateState(liftPosition,targetPosition);
            lift.setPower(motorPower);
            telemetry.addData("position: ",liftPosition);
            telemetry.addData("Power: ",motorPower);
            telemetry.addData("Error: ",PID.p_error);
            telemetry.update();
            sleep(time);
        }
    }
}
