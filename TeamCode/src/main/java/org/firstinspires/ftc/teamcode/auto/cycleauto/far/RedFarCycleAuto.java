package org.firstinspires.ftc.teamcode.auto.cycleauto.far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.auto.defaultauto.far.FarAutoPath;

@Autonomous(name = "Red \uD83D\uDD34 Far Auto PLAYOFFS \uD83C\uDFC6", group = "Playoffs")
public class RedFarCycleAuto extends FarAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
    @Override
    protected Location getFirstLocation() {
        return Location.FAR;
    }
}
