package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoSide.RedAuto;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;

@Autonomous(name="odometry")
public class newOpmode extends RedAuto {
    Drive robot = new Drive(this);
    Path points = new Path();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.doIMU();
        telemetry.addData("Waiting for", "start");
        telemetry.update();
        waitForStart();
        robot.startPositionTracking();
        points.addPoint(20,40);
        points.addPoint(40,0);
        telemetry.addData("Ready", "tracking started and points appended");
        telemetry.update();
        robot.runPath(points, 0.5, 2);
        while (opModeIsActive()) {

            telemetry.addData("robot x", robot.x());
            telemetry.addData("robot y", robot.y());
            telemetry.addData("running", robot.opMode.opModeIsActive());
            telemetry.addData("fl", robot.frontLeft.getPower());
            telemetry.addData("fr", robot.frontRight.getPower());
            telemetry.addData("bl", robot.backLeft.getPower());
            telemetry.addData("br", robot.backRight.getPower());
            telemetry.update();


        }
    }
}
