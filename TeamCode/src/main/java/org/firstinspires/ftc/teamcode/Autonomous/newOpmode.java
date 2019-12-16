package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoSide.BlueAuto;
import org.firstinspires.ftc.teamcode.AutoSide.RedAuto;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.PointArray;

@Autonomous(name="test")
public class newOpmode extends RedAuto {
    Drive robot = new Drive();
    PointArray points = new PointArray();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.doIMU();
        waitForStart();
        robot.startPositionTracking();
        points.appendPoint(1, 1);
        points.appendPoint(3, 7);
        robot.runPath(points, 1.0, 1.0);
        while(opModeIsActive()) {
            telemetry.addData("is active? ", isActive());
            telemetry.addData("Robot X ", robot.x());
            telemetry.addData("Robot Y ", robot.y());
            telemetry.addData("frontLeft power: ", robot.frontLeft.getPower());
            telemetry.addData("frontRight power: ", robot.frontRight.getPower());
            telemetry.addData("backLeft power: ", robot.backLeft.getPower());
            telemetry.addData("backRight power: ", robot.backRight.getPower());
            telemetry.update();
        }
    }
}
