package org.firstinspires.ftc.teamcode.auto.gatespamauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red \uD83D\uDD34 Close Gate SPAM \uFE0F Auto \uD83C\uDFC6", group = "Auto Gate Spam")
public class RedCloseGateSpamAuto extends CloseGateSpamAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
