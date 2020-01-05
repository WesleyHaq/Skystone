package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Point;
import org.firstinspires.ftc.teamcode.AutoSide.Auto;

@Autonomous(name="holo", group="bests")
public class basic_oton extends LinearOpMode {
    Drive robot = new Drive(this);
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.startPositionTracking();
        robot.reverse();
        robot.foundationServo.setPosition(0.3);
        while(robot.y()<2) {
            robot.pointDrive(new Point(0,2), 0.3);
        }
        while(robot.x()>-36) {
            robot.pointDrive(new Point(-36,2), 0.5);
        }
        while(robot.y()<32) {
            robot.pointDrive(new Point(-36,32), 0.5);
        }
        robot.foundationServo.setPosition(0);
        while(robot.y()>6) {
            robot.pointDrive(new Point(-36,6), 0.5);
        }
        robot.setAngle(0);
    }
}
