package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red \uD83D\uDD34 Robot 1 TeleOp", group = "Competition")
public class Robot1TeleOpRed extends Robot1TeleOpForNeel {
    @Override
    protected boolean isRed() {
        return true;
    }
}
