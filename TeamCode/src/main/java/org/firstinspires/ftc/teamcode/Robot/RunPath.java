package org.firstinspires.ftc.teamcode.Robot;

public class RunPath extends Thread {

    Drive robot;
    Path path;
    double power;
    double drift;

    public RunPath(Drive drive) {
        robot = drive;
    }

    public void setPath(Path path, double power, double drift) {
        this.path = path;
        this.power = power;
        this.drift = drift;
    }

    @Override
    public void run() {
        try {
            robot.runPath(path, power, drift);
        } catch (Exception e) {
            robot.opMode.telemetry.addData("exception thrown", e);
        }
    }
}
