package org.firstinspires.ftc.teamcode.auto.visionauto.far;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.cycleauto.close.CloseCycleAutoPath;

@Autonomous(name = "Blue \uD83D\uDD35 Far Vision \uD83D\uDC40 Auto", group = "Auto Vision")
public class BlueFarVisionAuto extends FarVisionAutoPath {
    @Override
    protected boolean isRed() {
        return false;
    }
    @Override
    protected boolean isFar() {
        return true;
    }
}
