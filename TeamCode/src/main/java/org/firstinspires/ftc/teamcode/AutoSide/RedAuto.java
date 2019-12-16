package org.firstinspires.ftc.teamcode.AutoSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class RedAuto extends LinearOpMode implements IAuto {
    public String getSide() {
        return "red";
    }
    public boolean isActive() {
        return opModeIsActive();
    }
}
