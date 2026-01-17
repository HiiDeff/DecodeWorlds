package org.firstinspires.ftc.teamcode.auto.cycleauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue \uD83D\uDD35 Close Cycle \uD83D\uDD04 Auto", group = "Auto Cycle")
public class BlueCloseCycleAuto extends CloseCycleAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
