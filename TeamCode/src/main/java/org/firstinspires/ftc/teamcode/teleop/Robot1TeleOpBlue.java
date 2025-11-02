package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue \uD83D\uDD35 Robot 1 TeleOp", group = "Competition")
public class Robot1TeleOpBlue extends Robot1TeleOpForNeel {
    @Override
    protected boolean isRed() {
        return false;
    }
}