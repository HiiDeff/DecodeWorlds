package org.firstinspires.ftc.teamcode.auto.cycleauto.far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.auto.defaultauto.far.FarAutoPath;

@Autonomous(name = "Blue \uD83D\uDD35 Far Cycle \uD83D\uDD04 Auto", group = "Auto Cycle")
public class BlueFarCycleAuto extends FarAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
    @Override
    protected Location getFirstLocation() {
        return Location.FAR;
    }
}
