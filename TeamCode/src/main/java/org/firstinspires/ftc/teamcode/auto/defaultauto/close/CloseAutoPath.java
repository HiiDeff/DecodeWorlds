package org.firstinspires.ftc.teamcode.auto.defaultauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class CloseAutoPath extends CloseAuto {
    public static double
            SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H,
            INTAKE_1_X, INTAKE_1_Y, INTAKE_1_H,  FORWARD_DIST_CYCLE_1,
            SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H, FORWARD_DIST_CYCLE_2,
            SHOOT_3_X, SHOOT_3_Y, SHOOT_3_H,
            INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H, FORWARD_DIST_CYCLE_3,
            SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H,
            PARK_X, PARK_Y, PARK_H,
            CONTROL_X, CONTROL_Y;

    static {
        SHOOT_1_X = 47.743; SHOOT_1_Y = -1.066; SHOOT_1_H = -2.35;
        INTAKE_1_X = 50.655; INTAKE_1_Y = -12.815; INTAKE_1_H = -Math.PI/2;
        FORWARD_DIST_CYCLE_1 = 29;
        SHOOT_2_X = 45.743; SHOOT_2_Y = -3.066; SHOOT_2_H = -2.35;
        INTAKE_2_X = 73.950; INTAKE_2_Y = -9.815; INTAKE_2_H = -Math.PI/2;
        FORWARD_DIST_CYCLE_2 = 30;
        SHOOT_3_X = 45.743; SHOOT_3_Y = -3.066; SHOOT_3_H = -2.35;
        INTAKE_3_X = 99.246; INTAKE_3_Y = -12.815; INTAKE_3_H = -Math.PI/2;
        FORWARD_DIST_CYCLE_3 = 34;
        SHOOT_4_X = 45.743; SHOOT_4_Y = -3.066; SHOOT_4_H = -2.35;
        PARK_X = 59; PARK_Y = -9; PARK_H = -Math.PI*7/8;
        CONTROL_X = 55; CONTROL_Y = 0;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(5.225,-29.996*getSign(),-2.249997*getSign());
    }

    @Override
    protected boolean isFar() {
        return false;
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
    protected Pose getControlPointPose() {
        return new Pose(CONTROL_X, CONTROL_Y);
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
