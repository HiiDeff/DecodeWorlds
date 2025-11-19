package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red \uD83D\uDD34 Robot 2 TeleOp", group = "Competition")
public class Robot2TeleOpRed extends Robot2TeleOp {
    @Override
    protected boolean isRed() {
        return true;
    }
}
