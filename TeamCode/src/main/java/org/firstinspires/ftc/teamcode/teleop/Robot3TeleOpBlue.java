package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue \uD83D\uDD35 Robot 3 TeleOp", group = "Competition")
public class Robot3TeleOpBlue extends Robot3TeleOp {
    @Override
    protected boolean isRed() {
        return false;
    }
}