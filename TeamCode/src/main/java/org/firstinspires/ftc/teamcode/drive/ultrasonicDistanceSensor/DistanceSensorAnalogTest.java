package org.firstinspires.ftc.teamcode.drive.ultrasonicDistanceSensor;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp
public class DistanceSensorAnalogTest extends LinearOpMode {

    private AnalogInput sensor;
    private DigitalChannel digital;
    @Override
    public void runOpMode() {
        sensor = hardwareMap.analogInput.get("ultrasonic1");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance: ",maxbotixRange());
            telemetry.update();
        }


    }
    private double maxbotixRange()
    {
        return sensor.getVoltage();
    }



}
