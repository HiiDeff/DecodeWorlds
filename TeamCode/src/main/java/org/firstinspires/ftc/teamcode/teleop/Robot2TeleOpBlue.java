package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue \uD83D\uDD35 Robot 2 TeleOp", group = "Competition")
public class Robot2TeleOpBlue extends Robot2TeleOp {
    @Override
    protected boolean isRed() {
        return false;
    }
}