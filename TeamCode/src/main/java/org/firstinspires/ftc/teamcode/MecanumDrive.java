package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumConfig;

import java.lang.Math;


public class MecanumDrive {

    MecanumConfig robot = new MecanumConfig();

    public MecanumDrive(MecanumConfig rob) {
        robot = rob;
    }


    //runtime is the timer
    private ElapsedTime runtime = new ElapsedTime();

    //lastAngles is used in the checkDirection() method
    public Orientation lastAngles = new Orientation();

    //globalAngle is used in getAngle()
    double globalAngle;

    //assume the robot is at coordinates (0,0)
    double fieldX = 0;
    double fieldY = 0;

    double vX = 0;
    double vY = 0;

    double angle;

    double ROBOT_WEIGHT_KG = 10;
    double WHEEL_FRICTION = 3;

    //calculate distance based on encoder counts
    public double encoderDistanceX() {
        double WHEEL_DIAMETER = 1.5;
        int COUNTS_PER_REVOLUTION = 1000;

        //The encoder will be plugged into the same port as the frontLeft motor
        return robot.frontLeft.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION;
    }
    public double encoderDistanceY() {
        double WHEEL_DIAMETER = 1.5;
        int COUNTS_PER_REVOLUTION = 1000;

        //The encoder will be plugged into the same port as the frontRight motor
        return robot.frontRight.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION;
    }

    public void resetPosition() {

    }

    //return calculated x and y positions

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }

    //returns calculated angle 0-360
    public double getAngle(MecanumConfig robot) {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = angles.firstAngle;

        if (globalAngle < 0) {
            globalAngle += 360;
        }

        return globalAngle;
    }

    public void velocityReset() {
        vX = encoderDistanceX();
        vY = encoderDistanceY();
    }
    public double getVelocity(double timeS) {
        double d1 = encoderDistanceX() - vX;
        double d2 = encoderDistanceY() - vY;
        double v = Math.sqrt(Math.pow(d1, 2) + Math.pow(d2, 2)) / timeS;
        return v;
    }

    public double smoothingFactor(double error, double velocity, double angularVelocity) {
        double linearSlip  = velocity * ROBOT_WEIGHT_KG - (WHEEL_FRICTION*0.75) * runtime.seconds();
        double angularSlip = ROBOT_WEIGHT_KG * Math.pow(velocity, 2) / ((velocity/angularVelocity * 360)/(2*3.14159265359));
        double slip = error - linearSlip;

        if (slip > 0.1) slip = 0.1;
        if (slip < -0.5) slip = -0.5;

        return slip;
    }

    // Sets power to drive at angle
    public void simpleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        robot.frontRight.setPower((Math.sin(Math.toRadians(angle)) * power));
        robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power));
        robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * power));
        robot.backLeft.setPower((Math.sin(Math.toRadians(angle)) * power));
    }

    //Calculates power to drive at angle with IMU correction
    public void angleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        robot.frontRight.setPower((Math.sin(Math.toRadians(angle)) * power));// - checkDirection());
        robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power));// + checkDirection());
        robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * power));// - checkDirection());
        robot.backLeft.setPower((Math.sin(Math.toRadians(angle)) * power));// + checkDirection());
    }

    public void relativeDrive(double angle, double distance, double power) {

        double d = Math.sqrt(Math.pow(encoderDistanceX(), 2) + Math.pow(encoderDistanceY(), 2));

        while (d < distance) {
            angleDrive(angle, power);
        }
    }

    //Drive to a point on the field
    public void pointDrive(double xPos, double yPos, double power, double timoutS) {
        double rX = xPos - fieldX;
        double rY = yPos - fieldY;
        double distance = Math.sqrt(Math.pow(rX, 2) + Math.pow(rY, 2));
        double angle = Math.tan(rY / rX);
        while (Math.abs(rX) < 1 && Math.abs(rY) < 1) {
            angleDrive(Math.toDegrees(angle), power);
        }
    }


    //IMU angle correction methods

    public double checkDirection() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        angle += deltaAngle;

        lastAngles = angles;


        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction;
        double gain = 0.025;

        //gain = mult * angle;

        correction = -angle * gain;        // reverse sign of angle for correction.

        return correction;
    }

    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void stop() {
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }

}
