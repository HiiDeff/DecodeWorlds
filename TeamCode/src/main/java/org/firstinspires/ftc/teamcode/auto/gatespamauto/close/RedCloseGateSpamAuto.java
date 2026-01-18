package org.firstinspires.ftc.teamcode.auto.gatespamauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red \uD83D\uDD34 Close Gate SPAM \uD83E\uDD69 Auto", group = "Auto Gate Spam")
public class RedCloseGateSpamAuto extends CloseGateSpamAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
