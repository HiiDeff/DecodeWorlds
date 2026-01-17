package org.firstinspires.ftc.teamcode.auto.defaultauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue \uD83D\uDD35 Close Auto", group = "Auto AA_Default")
public class BlueCloseAuto extends CloseAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
