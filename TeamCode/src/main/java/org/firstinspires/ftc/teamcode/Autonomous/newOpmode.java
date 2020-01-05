package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoSide.BlueAuto;
import org.firstinspires.ftc.teamcode.AutoSide.RedAuto;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;

@Autonomous(name="wes sly")
public class newOpmode extends BlueAuto {
    Drive robot = new Drive(this);
    Path points = new Path();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Waiting for", "start");
        telemetry.update();
        waitForStart();
        telemetry.update();
        robot.startPositionTracking();

        robot.foundationServo.setPosition(0.3);
        points.addPoint(0,2);
        points.addPoint(-16, 2);
        points.addPoint(-16, 32);
        telemetry.addData("hi bob!", "hello!");
        robot.runPath(points, 0.5, 2);
        points.clear();
        points.addPoint(-32, 24);
        points.addPoint(-30, 16, 35);
        points.addPoint(-26, 6, 90);
        points.addPoint(-42, 12, 90);
        robot.runPath(points, 0.5, 2);
        telemetry.addData("Ready", "tracking started and points appended");
        telemetry.update();
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
