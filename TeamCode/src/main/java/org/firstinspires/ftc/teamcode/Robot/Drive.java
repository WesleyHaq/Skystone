package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutoSide.BlueAuto;
import org.firstinspires.ftc.teamcode.AutoSide.IAuto;

import java.lang.Math;


public class Drive extends Config {

    //runtime is the timer
    private ElapsedTime runtime = new ElapsedTime();

    //lastAngles is used in the checkDirection() method
    public Orientation lastAngles = new Orientation();

    PositionTracking tracker = new PositionTracking();

    //IMUAngle is used in getAngle()
    double IMUAngle;

    //assume the robot is at coordinates (0,0)
    Point robot = new Point(0,0);

    double velX = 0;
    double velY = 0;

    //current angle in radians
    private double currentAngle = 0;
    //variable used for IMU direction correction
    double checkAngle;

    //not used yet. Maybe later.
    double ROBOT_WEIGHT_KG = 10;
    double WHEEL_FRICTION = 3;

    //calculate distance based on encoder counts
    public double encoderDistanceY1() {
        double WHEEL_DIAMETER = 1.5;
        int COUNTS_PER_REVOLUTION = 1000;

        //The encoder will be plugged into the same port as the frontLeft motor
        return frontLeft.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION;
    }
    public double encoderDistanceY2() {
        double WHEEL_DIAMETER = 1.5;
        int COUNTS_PER_REVOLUTION = 1000;

        //The encoder will be plugged into the same port as the frontRight motor
        return frontRight.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION;
    }

    public double encoderDistanceX() {
        double WHEEL_DIAMETER = 1.5;
        int COUNTS_PER_REVOLUTION = 1000;

        //The encoder will be plugged into the same port as the frontRight motor
        return backLeft.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION;
    }

    public double encoderDistanceY() {
        return (encoderDistanceY1() + encoderDistanceY2())/2;
    }

    public void startPositionTracking() {
        tracker.start();
    }

    public void resetPosition() {
        tracker.interrupt();
    }

    //return calculated x and y positions

    public double x() {
        return robot.x;
    }

    public double y() {
        return robot.y;
    }

    public double getVelX() {
        return velX;
    }
    
    public double getVelY() {
        return velY;
    }
    
    //returns calculated angle 0-360
    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        IMUAngle = angles.firstAngle;

        if (IMUAngle < 0) {
            IMUAngle += 360;
        }

        return IMUAngle;
    }

    double vX;
    double vY;
    
    public void velocityReset() {
        vX = encoderDistanceX();
        vY = encoderDistanceY();
        runtime.reset();
    }
    public double getVelocity() {
        double d1 = encoderDistanceX() - vX;
        double d2 = encoderDistanceY() - vY;
        double v = Math.sqrt(Math.pow(d1, 2) + Math.pow(d2, 2)) / runtime.seconds();
        return v;
    }

    // Sets power to drive at angle
    public void simpleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        frontRight.setPower((Math.sin(Math.toRadians(angle)) * power));
        frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power));
        backRight.setPower((Math.cos(Math.toRadians(angle)) * power));
        backLeft.setPower((Math.sin(Math.toRadians(angle)) * power));
    }

    //Calculates power to drive at angle with IMU correction
    private void angleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        frontRight.setPower(Math.sin(Math.toRadians(angle)) * power - checkDirection());
        frontLeft.setPower(Math.cos(Math.toRadians(angle)) * power + checkDirection());
        backRight.setPower(Math.cos(Math.toRadians(angle)) * power - checkDirection());
        backLeft.setPower(Math.sin(Math.toRadians(angle)) * power + checkDirection());
    }

    public void relativeDrive(double angle, double distance, double power) {

        double d = Math.sqrt(Math.pow(encoderDistanceX(), 2) + Math.pow(encoderDistanceY(), 2));

        while (d < distance) {
            d = Math.sqrt(Math.pow(encoderDistanceX(), 2) + Math.pow(encoderDistanceY(), 2));
            angleDrive(angle, power);
        }
    }


    //Drive to a point on the robot
    public void pointDrive(double xPos, double yPos, double power, double timoutS) {
        double rX = xPos - robot.x;
        double rY = yPos - robot.y;
        double distance = Math.sqrt(Math.pow(rX, 2) + Math.pow(rY, 2));
        double angle = Math.atan(rY / rX);
        angleDrive(Math.toDegrees(angle), power);
    }

    public void runPath(PointArray pointList, double power, double drift) {
        boolean driving = true;
        double accel = 0;
        double velX = 0;
        double velY = 0;
        double lastVelX = 0;
        double lastVelY = 0;
        double lastX = 0;
        double lastY = 0;
        double driveAngle;
        double pathSlope;
        double crossDist;

        Point target = new Point(0,0);
        Point stopPoint = new Point(0,0);

        Line pathLine = new Line(0, 0);
        Line robotLine = new Line(0, 0);

        int currentPoint = 0;
        Point pointOne = pointList.get(0);
        Point pointTwo = pointList.get(1);

        while (driving) {

            target.x = pointTwo.x;
            target.y = pointTwo.y;

            pathSlope = (pointTwo.y - pointOne.y) / (pointTwo.x - pointOne.x);

            pathLine.setLine(pointOne, pointTwo);
            robotLine.setLine(robot, -1/pathSlope);

            stopPoint.setX(0.5 * (Math.pow(velX, 2) * drift));
            stopPoint.setY(0.5 * (Math.pow(velY, 2) * drift));

            target.x -= stopPoint.x;
            target.y -= stopPoint.y;


            crossDist = Math.sqrt(Math.pow((robot.x - pathLine.intersection(robotLine).x), 2) + Math.pow(robot.y - pathLine.intersection(robotLine).y, 2));
            driveAngle = (Math.atan(target.y / target.x)/drift + (-1/pathSlope)*crossDist*drift)/(drift + 1/drift);
            angleDrive(driveAngle, power);



            if (robot.y+stopPoint.y - pointTwo.y == pathSlope * (robot.x+stopPoint.x - pointTwo.x)) {

                if (pointOne == pointList.get(pointList.length())) {
                    driving = false;
                } else {
                    currentPoint++;
                    pointOne = pointList.get(currentPoint);
                    pointTwo = pointList.get(currentPoint + 1);
                }
            }
        }


}


    //IMU angle correction methods

    public double checkDirection() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        checkAngle += deltaAngle;

        lastAngles = angles;


        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction;
        double gain = 0.025;

        //gain = mult * angle;

        correction = -checkAngle * gain;        // reverse sign of angle for correction.

        return correction;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        IMUAngle = 0;
    }

    public void stop() {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }


    protected class PositionTracking extends Thread {
        public void run() {
            try {
                double robotX = 0;
                double robotY = 0;
                
                double lastEncoderX = 0;
                double lastEncoderY = 0;

                double deltaEncoderX;
                double deltaEncoderY;

                double deltaX;
                double deltaY;

                double lastVelX = 0;
                double lastVelY = 0;

                final double encoderRadius = 14;

                int loopEvent = 0;

                while(!Thread.interrupted()) {
                    currentAngle = (encoderDistanceY1() - encoderDistanceY2())/(2*encoderRadius);

                    deltaEncoderX = encoderDistanceX() - lastEncoderX;
                    deltaEncoderY = encoderDistanceY() - lastEncoderY;

                    deltaY = Math.sin(currentAngle) * deltaEncoderY + Math.sin(currentAngle - Math.PI/2) * deltaEncoderX;
                    deltaX = Math.cos(currentAngle) * deltaEncoderY + Math.cos(currentAngle - Math.PI/2) * deltaEncoderX;

                    robotX += deltaX;
                    robotY += deltaY;
                    robot.setPoint(robotX, robotY);

                    if(loopEvent%5==0) {
                        velX = (robot.x - lastVelX) * 20;
                        velY = (robot.y - lastVelY) * 20;
                        lastVelX = robot.x;
                        lastVelY = robot.y;
                    }

                    lastEncoderX = encoderDistanceX();
                    lastEncoderY = encoderDistanceY();

                    loopEvent++;

                    try {
                        Thread.sleep(10);
                    } catch(InterruptedException e) {
                        return;
                    }
                }

            } catch(Exception e) {
                //exception!
            }
        }
    }
}

