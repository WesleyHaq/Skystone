package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    double position = 0.25;

    boolean pressed = false;

    double power = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.motorsOn();

        waitForStart();

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

            robot.intake1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            robot.intake2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            robot.lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

            if (gamepad2.left_bumper && position < 1) {
                position += 0.03;
            }
            if (gamepad2.right_bumper && position > 0.25) {
                position -= 0.03;
            }

            if (gamepad2.x) {
                robot.leftGrip.setPosition(0.75);
                robot.rightGrip.setPosition(0.25);
            }
            else {
                robot.leftGrip.setPosition(0.9);
                robot.rightGrip.setPosition(0.1);
            }

            robot.leftBase.setPosition(-position + 1);
            robot.rightBase.setPosition(position);
            robot.leftStabilization.setPosition(position*1.15 - 0.05);
            robot.rightStabilization.setPosition(-position*1.15+1.05);

            robot.frontLeft.setPower(frontLeft);
            robot.frontRight.setPower(frontRight);
            robot.backLeft.setPower(backLeft);
            robot.backRight.setPower(backRight);

            telemetry.addData("robot x", robot.x());
            telemetry.addData("robot y", robot.y());
            telemetry.addData("vel x", robot.getVelX());
            telemetry.addData("vel y", robot.getVelY());
            telemetry.addData("fl", robot.frontLeft.getPower());
            telemetry.addData("fr", robot.frontRight.getPower());
            telemetry.addData("bl", robot.backLeft.getPower());
            telemetry.addData("br", robot.backRight.getPower());
            telemetry.update();

            sleep(50);
        }
    }

}
