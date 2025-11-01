package org.firstinspires.ftc.teamcode.auto.defaultauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class FarAutoPath extends FarAuto {
    public static double
            SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H,
            INTAKE_1_X, INTAKE_1_Y, INTAKE_1_H, FORWARD_DIST_CYCLE_1,
            SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H, FORWARD_DIST_CYCLE_2,
            SHOOT_3_X, SHOOT_3_Y, SHOOT_3_H,
            INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H, FORWARD_DIST_CYCLE_3,
            SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_1_X = 7.67; SHOOT_1_Y = -3.46; SHOOT_1_H = 0.42486;
        INTAKE_1_X = 27.71; INTAKE_1_Y = 12.5; INTAKE_1_H = Math.toRadians(90);
        FORWARD_DIST_CYCLE_1 = 34;
        SHOOT_2_X = 7.67; SHOOT_2_Y = -3.46; SHOOT_2_H = 0.42486;
        INTAKE_2_X = 50.94; INTAKE_2_Y = 12.5; INTAKE_2_H = Math.toRadians(90);
        FORWARD_DIST_CYCLE_2 = 33;
        SHOOT_3_X = 7.67; SHOOT_3_Y = -3.46; SHOOT_3_H = 0.42486;
        INTAKE_3_X = 74.58; INTAKE_3_Y = 12.5; INTAKE_3_H = Math.toRadians(90);
        FORWARD_DIST_CYCLE_3 = 28;
        SHOOT_4_X = 7.67; SHOOT_4_Y = -3.46; SHOOT_4_H = 0.42486;
        PARK_X = 18; PARK_Y = -3.5; PARK_H = 0.40486;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.toRadians(0));
    }

    @Override
    protected boolean isFar() {
        return true;
    }

    @Override
    protected Pose getShoot1Pose() {
        return new Pose(SHOOT_1_X, SHOOT_1_Y*getSign(), SHOOT_1_H*getSign());
    }

    @Override
    protected Pose getIntake1Pose() {
        return new Pose(INTAKE_1_X, INTAKE_1_Y*getSign(), INTAKE_1_H*getSign());
    }

    @Override
    protected Pose getShoot2Pose() {
        return new Pose(SHOOT_2_X, SHOOT_2_Y*getSign(), SHOOT_2_H*getSign());
    }

    @Override
    protected Pose getIntake2Pose() {
        return new Pose(INTAKE_2_X, INTAKE_2_Y*getSign(), INTAKE_2_H*getSign());
    }

    @Override
    protected Pose getShoot3Pose() {
        return new Pose(SHOOT_3_X, SHOOT_3_Y*getSign(), SHOOT_3_H*getSign());
    }

    @Override
    protected Pose getIntake3Pose() {
        return new Pose(INTAKE_3_X, INTAKE_3_Y*getSign(), INTAKE_3_H*getSign());
    }

    @Override
    protected Pose getShoot4Pose() {
        return new Pose(SHOOT_4_X, SHOOT_4_Y*getSign(), SHOOT_4_H*getSign());
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    @Override
    protected double getIntakeForwardDist(int cycleNumber) {
        if(cycleNumber==1) {
            return FORWARD_DIST_CYCLE_1;
        } else if(cycleNumber==2) {
            return FORWARD_DIST_CYCLE_2;
        } else {
            return FORWARD_DIST_CYCLE_3;
        }
    }
}
