package org.firstinspires.ftc.teamcode.auto.cycleauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.defaultauto.close.CloseAutoPath;

@Config
@Autonomous(name = "Red \uD83D\uDD34 Far Auto PLAYOFFS \uD83C\uDFC6", group = "Competition")
public class RedFarCycleAuto extends CloseAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
