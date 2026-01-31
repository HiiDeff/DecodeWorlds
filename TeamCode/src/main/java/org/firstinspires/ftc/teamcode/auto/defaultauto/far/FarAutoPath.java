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
        SHOOT_START_X = -11.778; SHOOT_START_Y = 4; SHOOT_START_H = -2.75;
        SHOOT_MID_X = -11.778; SHOOT_MID_Y = 4; SHOOT_MID_H = -2.72;
        SHOOT_FAR_X = -11.778; SHOOT_FAR_Y = 4; SHOOT_FAR_H = -2.70;
        SHOOT_LOADING_X = -11.778; SHOOT_LOADING_Y = 4; SHOOT_LOADING_H = -2.72;

        INTAKE_MID_X = -51.574; INTAKE_MID_Y = -10.916; INTAKE_1_H = -Math.PI/2;
        INTAKE_MID_FORWARD_X = -51.574; INTAKE_MID_FORWARD_Y = -29.444; INTAKE_MID_FORWARD_H = -Math.PI/2;

        INTAKE_FAR_X = -27.974; INTAKE_FAR_Y = -9.309; INTAKE_FAR_H = -Math.PI/2;
        INTAKE_FAR_FORWARD_X = -27.974; INTAKE_FAR_FORWARD_Y = -28.526; INTAKE_FAR_FORWARD_H = -Math.PI/2;

        INTAKE_LOADING_X = -17.107; INTAKE_LOADING_Y = -40; INTAKE_LOADING_H = -0.547;
        INTAKE_LOADING_FORWARD_X = -4.334; INTAKE_LOADING_FORWARD_Y = -41; INTAKE_LOADING_FORWARD_H = -1.260;

        GATE_X = -55.975; GATE_Y = -40.256; GATE_H = 0;
        PARK_X = -8; PARK_Y = -8; PARK_H = -Math.PI/2;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0, Math.PI);
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
