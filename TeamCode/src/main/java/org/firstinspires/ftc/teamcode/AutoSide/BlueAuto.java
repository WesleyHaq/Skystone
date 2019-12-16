package org.firstinspires.ftc.teamcode.AutoSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BlueAuto extends LinearOpMode implements IAuto {
    public String getSide() {
        return "blue";
    }
    public boolean isActive() {
        return opModeIsActive();
    }
}
