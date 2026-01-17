package org.firstinspires.ftc.teamcode.auto.gatespamauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue \uD83D\uDD35 Close Gate SPAM \uFE0F Auto \uD83C\uDFC6", group = "Auto Gate Spam")
public class BlueCloseGateSpamAuto extends CloseGateSpamAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
