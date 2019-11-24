package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "New TeleOP")
public class MecanumTeleOp extends LinearOpMode {

    MecanumConfig robot = new MecanumConfig();

    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double side;
    double oldAngle;

    boolean pressed = false;

    double power = 0.5;
    boolean inverted;
    boolean released = true;

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
