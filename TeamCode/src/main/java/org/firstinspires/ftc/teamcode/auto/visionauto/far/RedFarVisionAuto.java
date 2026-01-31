package org.firstinspires.ftc.teamcode.auto.visionauto.far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.cycleauto.close.CloseCycleAutoPath;

@Autonomous(name = "Red \uD83D\uDD34 Far Vision \uD83D\uDC40 Auto", group = "Auto Vision")
public class RedFarVisionAuto extends FarVisionAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
    @Override
    protected boolean isFar() {
        return true;
    }
}
