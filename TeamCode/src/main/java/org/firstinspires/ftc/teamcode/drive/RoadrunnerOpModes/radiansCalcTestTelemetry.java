package org.firstinspires.ftc.teamcode.drive.RoadrunnerOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Math.toRadians;

@TeleOp
public class radiansCalcTestTelemetry extends LinearOpMode {




    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("180 in rad", toRadians(180));
            telemetry.update();
        }
    }
}
