package org.firstinspires.ftc.teamcode.auto.defaultauto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
@Autonomous(name = "Red \uD83D\uDD34 Close Auto", group = "Competition")
public class RedCloseAuto extends CloseAuto {
    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return null;
    }

    @Override
    protected boolean isRed() {
        return true;
    }

    @Override
    protected Pose getShoot1Pose() {
        return null;
    }

    @Override
    protected Pose getIntake1Pose() {
        return null;
    }

    @Override
    protected Pose getShoot2Pose() {
        return null;
    }

    @Override
    protected Pose getIntake2Pose() {
        return null;
    }

    @Override
    protected Pose getShoot3Pose() {
        return null;
    }

    @Override
    protected Pose getIntake3Pose() {
        return null;
    }

    @Override
    protected Pose getShoot4Pose() {
        return null;
    }
}
