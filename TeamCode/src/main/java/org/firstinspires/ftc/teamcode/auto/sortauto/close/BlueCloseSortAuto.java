package org.firstinspires.ftc.teamcode.auto.sortauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue \uD83D\uDD35 Close SORT \uD83E\uDD39 Auto", group = "Auto Sort")
public class BlueCloseSortAuto extends CloseSortAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
