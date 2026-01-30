package org.firstinspires.ftc.teamcode.auto.sortauto.close;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red \uD83D\uDD34 Close SORT \uD83E\uDD39 Auto", group = "Auto Sort")
public class RedCloseSortAuto extends CloseSortAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
