package org.firstinspires.ftc.teamcode.AutoSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Auto extends LinearOpMode {
    public boolean isActive() {
        return opModeIsActive();
    }
    public abstract String getSide();
}
