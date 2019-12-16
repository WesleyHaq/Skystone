package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Config;

@TeleOp(name = "Phantom TeleOp")
public class PhantomTeleOp extends LinearOpMode {

    Config robot = new Config();

    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double side;

    boolean pressed = false;

    double power = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
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
            telemetry.update();

            side = (gamepad1.left_stick_x + gamepad1.right_stick_x) / -2;
            frontRight = -gamepad1.left_stick_y + side;
            backRight = -gamepad1.left_stick_y - side;
            frontLeft = -gamepad1.right_stick_y - side;
            backLeft = -gamepad1.right_stick_y + side;

            if (gamepad1.dpad_left) {

                frontLeft = -1 - gamepad1.right_trigger;
                frontRight = 1 * (1 - gamepad1.right_trigger);
                backLeft = 1 * (1 - gamepad1.right_trigger);
                backRight = -1 - gamepad1.right_trigger;

            } else if (gamepad1.dpad_right) {

                frontLeft = 1 * (1 - gamepad1.right_trigger);
                frontRight = -1 - gamepad1.right_trigger;
                backLeft = -1 - gamepad1.right_trigger;
                backRight = 1 * (1 - gamepad1.right_trigger);
            }

        }
    }

}
