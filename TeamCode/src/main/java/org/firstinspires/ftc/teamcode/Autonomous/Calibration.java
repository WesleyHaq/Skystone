package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Drive;

public class Calibration extends LinearOpMode {
    Drive robot = new Drive(this);
    Orientation angles = new Orientation();
    public void runOpMode() {
        robot.motorsOn();
        robot.doIMU();
        waitForStart();
        robot.startPositionTracking();

    }
}
