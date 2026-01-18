package org.firstinspires.ftc.teamcode.auto.gatespamauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue \uD83D\uDD35 Close Gate SPAM \uD83E\uDD69 Auto", group = "Auto Gate Spam")
public class BlueCloseGateSpamAuto extends CloseGateSpamAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
