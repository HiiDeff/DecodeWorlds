package org.firstinspires.ftc.teamcode.auto.cycleauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red \uD83D\uDD34 Close Cycle \uD83D\uDD04 Auto", group = "Auto Cycle")
public class RedCloseCycleAuto extends CloseCycleAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
