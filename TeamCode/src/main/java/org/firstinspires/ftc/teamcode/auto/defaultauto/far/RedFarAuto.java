package org.firstinspires.ftc.teamcode.auto.defaultauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Autonomous(name = "Red \uD83D\uDD34 Far Auto", group = "Competition")
public class RedFarAuto extends FarAutoPath {
    @Override
    protected boolean isRed() {
        return true;
    }
}
