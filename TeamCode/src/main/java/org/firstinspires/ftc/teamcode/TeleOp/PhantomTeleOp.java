package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Config;
import org.firstinspires.ftc.teamcode.Robot.Drive;

@TeleOp(name = "Phantom TeleOp")
public class PhantomTeleOp extends LinearOpMode {

    Drive robot = new Drive(this);

    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double side;
    double position = 0.19;
    boolean grip;
    double lift;


    boolean pressed = false;

    double power = 0.5;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        robot.motorsOn();
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.startPositionTracking();

        while (!isStopRequested()) {
            if (gamepad1.right_bumper && !pressed && power < 1) {
                power += 0.25;
                pressed = true;
            }
            if (gamepad1.left_bumper && !pressed && power > 0.25) {
                power -= 0.25;
                pressed = true;
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                pressed = false;
            }

            telemetry.addData("Power:", power);

            side = (gamepad1.left_stick_x + gamepad1.right_stick_x) / -2;
            frontRight = -gamepad1.right_stick_y + side;
            backRight = -gamepad1.right_stick_y - side;
            frontLeft = -gamepad1.left_stick_y - side;
            backLeft = -gamepad1.left_stick_y + side;

            robot.frontLeft.setPower(frontLeft*power);
            robot.frontRight.setPower(frontRight*power);
            robot.backLeft.setPower(backLeft*power);
            robot.backRight.setPower(backRight*power);

            robot.intake1.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.75);
            robot.intake2.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.75);

            if(robot.lift.getCurrentPosition() > -10 && -gamepad2.right_stick_y < 0) {
                lift = -gamepad2.right_stick_y/4;
            }
            else if((robot.lift.getCurrentPosition() < 640 && -gamepad2.right_stick_y > 0)) {
                lift = -gamepad2.right_stick_y;
            } else if (robot.lift.getCurrentPosition() < -20) {
                lift = 0.1;
            } else if (robot.lift.getCurrentPosition() > 640) {
                lift = -0.1;
            } else {
                lift = 0;
            }
            robot.lift.setPower(lift);
            telemetry.addData("lift", robot.lift.getCurrentPosition());

            if ((position > 0.19 && gamepad2.left_stick_x < 0) || (position < 1 && gamepad2.left_stick_x > 0)) {
                position += gamepad2.left_stick_x * 0.05;
                if(position < 0.19) {
                    position = 0.19;
                }
            }

            if(position < 0.24) {
                grip = false;
            } else {
                grip = true;
            }

            if (gamepad2.x) {
                grip = !grip;
            }

            if (grip) {
                robot.leftGrip.setPosition(0.55);
                robot.rightGrip.setPosition(0.45);
            }
            else {
                robot.leftGrip.setPosition(0.87);
                robot.rightGrip.setPosition(0.07);
            }

            if(gamepad1.dpad_down) {
                robot.foundationServo.setPosition(0);
            }
            if(gamepad1.dpad_up) {
                robot.foundationServo.setPosition(0.3);
            }

            robot.leftBase.setPosition(-(Math.abs(position-0.24)+0.24) + 1);
            robot.rightBase.setPosition((Math.abs(position-0.24)+0.24));
            robot.leftStabilization.setPosition((Math.abs(position-0.24)+0.24)*1.13 - 0.07);
            robot.rightStabilization.setPosition(-(Math.abs(position-0.24)+0.24)*1.13 + 1.07);



            telemetry.addData("robot x", robot.x());
            telemetry.addData("robot y", robot.y());
            telemetry.addData("angle", robot.getAngle());
            telemetry.addData("vel x", robot.getVelX());
            telemetry.addData("vel y", robot.getVelY());
            telemetry.addData("fl", robot.frontLeft.getPower());
            telemetry.addData("fr", robot.frontRight.getPower());
            telemetry.addData("bl", robot.backLeft.getPower());
            telemetry.addData("br", robot.backRight.getPower());
            telemetry.update();
        }
    }

}
