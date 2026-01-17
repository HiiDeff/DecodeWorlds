package org.firstinspires.ftc.teamcode.auto.defaultauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red \uD83D\uDD34 Close Auto", group = "Auto AA_Default")
public class RedCloseAuto extends CloseAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
