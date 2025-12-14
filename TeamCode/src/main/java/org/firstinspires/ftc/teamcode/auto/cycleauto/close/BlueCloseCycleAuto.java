package org.firstinspires.ftc.teamcode.auto.cycleauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.cycleauto.close.CloseCycleAutoPath;

@Config
@Autonomous(name = "Blue \uD83D\uDD35 Close Auto PLAYOFFS \uD83C\uDFC6", group = "Competition")
public class BlueCloseCycleAuto extends CloseCycleAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
}
