package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Point;
import org.firstinspires.ftc.teamcode.AutoSide.Auto;

@Autonomous(name="holo", group="bests")
public class basic_oton extends LinearOpMode {
    Drive robot = new Drive(this);
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.foundationServo.setPosition(0);
        robot.doIMU();
        telemetry.addData("IMU status", "calibrating");
        telemetry.update();
        while(opModeIsActive() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("imu status", "calibrated done, waiting for start");
        telemetry.update();
        waitForStart();
        robot.motorsOn();
        robot.zeroBrake();
        robot.startPositionTracking();
        robot.foundationServo.setPosition(0.3);
        while(robot.encoderDistanceY()>-2 && opModeIsActive()) {
            robot.frontLeft.setPower(-0.3);
            robot.frontRight.setPower(-0.3);
            robot.backLeft.setPower(-0.3);
            robot.backRight.setPower(-0.3);
        }
        while(robot.encoderDistanceX()<10 && opModeIsActive()) {
            robot.frontLeft.setPower(0.5);
            robot.frontRight.setPower(-0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(0.5);
        }
        while(robot.encoderDistanceY()>-32 && opModeIsActive()) {
            robot.frontLeft.setPower(-0.5);
            robot.frontRight.setPower(-0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(-0.5);
        }
        robot.stop();
        sleep(250);
        robot.foundationServo.setPosition(0);
        sleep(1000);
        while(robot.encoderDistanceY()<-18 && opModeIsActive()) {
            robot.frontLeft.setPower(0.5);
            robot.frontRight.setPower(0.7);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(0.7);
        }
        while(robot.getAngle() > -90 && opModeIsActive()) {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0.7);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0.7);
        }
        robot.stop();
        robot.foundationServo.setPosition(0.3);
        sleep(100);
        while(robot.y() > -18) {
            robot.frontLeft.setPower(-0.5);
            robot.frontRight.setPower(0.5);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(-0.5);
        }
        while(robot.x() < 10) {
            robot.frontLeft.setPower(-0.5);
            robot.frontRight.setPower(-0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(-0.5);
        }
        while(robot.x() > -34) {
            robot.frontLeft.setPower(0.5);
            robot.frontRight.setPower(0.5);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(0.5);
        }
    }
}
