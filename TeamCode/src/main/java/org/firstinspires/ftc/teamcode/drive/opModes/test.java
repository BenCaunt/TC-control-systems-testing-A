package org.firstinspires.ftc.teamcode.drive.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.templates.autoTemplate;
@Config
@Autonomous
public class test extends autoTemplate {
    final double inches = 48;
    @Override
    public void runOpMode() {
        SetUp();
        telemetry.addData("Ready: ","yes");
        telemetry.update();
        waitForStart();
        encoderDriveIMU(inches);
    }
}
