package org.firstinspires.ftc.teamcode.auto.defaultauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class FarAutoPath extends FarAuto {
    public static double
            SHOOT_MID_X, SHOOT_MID_Y, SHOOT_MID_H,
            INTAKE_MID_X, INTAKE_MID_Y, INTAKE_1_H,
            INTAKE_MID_FORWARD_X, INTAKE_MID_FORWARD_Y, INTAKE_MID_FORWARD_H,
            SHOOT_FAR_X, SHOOT_FAR_Y, SHOOT_FAR_H,
            INTAKE_FAR_X, INTAKE_FAR_Y, INTAKE_FAR_H,
            INTAKE_FAR_FORWARD_X, INTAKE_FAR_FORWARD_Y, INTAKE_FAR_FORWARD_H,
            SHOOT_LOADING_X, SHOOT_LOADING_Y, SHOOT_LOADING_H,
            INTAKE_LOADING_X, INTAKE_LOADING_Y, INTAKE_LOADING_H,
            INTAKE_LOADING_FORWARD_X, INTAKE_LOADING_FORWARD_Y, INTAKE_LOADING_FORWARD_H,
            SHOOT_START_X, SHOOT_START_Y, SHOOT_START_H,
            GATE_X, GATE_Y, GATE_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_START_X = -8.493; SHOOT_START_Y = -0.686; SHOOT_START_H = -2.76;
        SHOOT_MID_X = -8.493; SHOOT_MID_Y = -0.686; SHOOT_MID_H = -2.76;
        SHOOT_FAR_X = -8.493; SHOOT_FAR_Y = -0.686; SHOOT_FAR_H = -2.75;
        SHOOT_LOADING_X = -8.493; SHOOT_LOADING_Y = -0.686; SHOOT_LOADING_H = -2.76;

        INTAKE_MID_X = -47.394; INTAKE_MID_Y = -15; INTAKE_1_H = -Math.PI/2;
        INTAKE_MID_FORWARD_X = -49.394; INTAKE_MID_FORWARD_Y = -33; INTAKE_MID_FORWARD_H = -Math.PI/2;

        INTAKE_FAR_X = -24; INTAKE_FAR_Y = -14; INTAKE_FAR_H = -Math.PI/2;
        INTAKE_FAR_FORWARD_X = -27; INTAKE_FAR_FORWARD_Y = -36; INTAKE_FAR_FORWARD_H = -Math.PI/2;

//        INTAKE_LOADING_X = -6.206; INTAKE_LOADING_Y = -14.586; INTAKE_LOADING_H = -1.145;
        INTAKE_LOADING_X = -11.510; INTAKE_LOADING_Y = -43.842; INTAKE_LOADING_H = -1.372;
        INTAKE_LOADING_FORWARD_X = -8.052; INTAKE_LOADING_FORWARD_Y = -44.875; INTAKE_LOADING_FORWARD_H = -1.459;

        GATE_X = -56.620; GATE_Y = -39.834; GATE_H = 0;
        PARK_X = -25.113; PARK_Y = -5.868; PARK_H = 0.390-Math.PI;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.PI);
    }

    @Override
    protected boolean isFar() {
        return false;
    }

    @Override
    protected Pose getShootStartPose() {
        return new Pose(SHOOT_START_X, SHOOT_START_Y*getSign(), SHOOT_START_H*getSign());
    }

    @Override
    protected Pose getIntakeMidPose() {
        return new Pose(INTAKE_MID_X, INTAKE_MID_Y*getSign(), INTAKE_1_H*getSign());
    }

    @Override
    protected Pose getShootMidPose() {
        return new Pose(SHOOT_MID_X, SHOOT_MID_Y*getSign(), SHOOT_MID_H*getSign());
    }

    @Override
    protected Pose getIntakeFarPose() {
        return new Pose(INTAKE_FAR_X, INTAKE_FAR_Y *getSign(), INTAKE_FAR_H *getSign());
    }

    @Override
    protected Pose getShootFarPose() {
        return new Pose(SHOOT_FAR_X, SHOOT_FAR_Y*getSign(), SHOOT_FAR_H*getSign());
    }

    @Override
    protected Pose getIntakeLoadingZonePose() {
        return new Pose(INTAKE_LOADING_X, INTAKE_LOADING_Y *getSign(), INTAKE_LOADING_H *getSign());
    }

    @Override
    protected Pose getShootLoadingZonePose() {
        return new Pose(SHOOT_LOADING_X, SHOOT_LOADING_Y*getSign(), SHOOT_LOADING_H*getSign());
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    protected Pose getIntakeMidForwardPose() {return new Pose(INTAKE_MID_FORWARD_X,  INTAKE_MID_FORWARD_Y *getSign(), INTAKE_MID_FORWARD_H *getSign());}

    @Override
    protected Pose getIntakeFarForwardPose() {return new Pose(INTAKE_FAR_FORWARD_X, INTAKE_FAR_FORWARD_Y *getSign(), INTAKE_FAR_FORWARD_H *getSign());}

    @Override
    protected Pose getIntakeLoadingZoneForwardPose() {return new Pose(INTAKE_LOADING_FORWARD_X, INTAKE_LOADING_FORWARD_Y *getSign(), INTAKE_LOADING_FORWARD_H *getSign());}

    @Override
    protected Pose getGatePose() {
        return new Pose(GATE_X, GATE_Y*getSign(), GATE_H*getSign());
    }
}
