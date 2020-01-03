package org.firstinspires.ftc.teamcode.AutoSide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Auto extends LinearOpMode {
    public boolean isActive(Auto opMode) {
        return opMode.opModeIsActive();
    }
    public String getSide(Auto opMode) {
        if(opMode instanceof BlueAuto) {
            return "blue";
        }
        else if(opMode instanceof RedAuto) {
            return "red";
        }
        else {
            return null;
        }
    }
}
