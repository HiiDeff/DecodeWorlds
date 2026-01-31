package org.firstinspires.ftc.teamcode.auto.visionauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class FarVisionAutoPath extends FarVisionAuto {
    public static double
            SHOOT_START_X, SHOOT_START_Y, SHOOT_START_H,
            LOADING_ZONE_INTAKE_X, LOADING_ZONE_INTAKE_Y, LOADING_ZONE_INTAKE_H,
            GATE_INTAKE_X, GATE_INTAKE_Y, GATE_INTAKE_H,
            LOADING_ZONE_FORWARD_X, LOADING_ZONE_FORWARD_Y, LOADING_ZONE_FORWARD_H,
            GATE_FORWARD_X, GATE_FOWARD_Y, GATE_FOWARD_H,

            GATE_X, GATE_Y, GATE_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_START_X = -11.778; SHOOT_START_Y = 4; SHOOT_START_H = -2.75;

        LOADING_ZONE_INTAKE_X = 0.0; LOADING_ZONE_INTAKE_Y = 0.0; LOADING_ZONE_INTAKE_H = 0.0;
        GATE_INTAKE_X = 0.0; GATE_INTAKE_Y = 0.0; GATE_INTAKE_H = 0.0;
        LOADING_ZONE_FORWARD_X = 0.0; LOADING_ZONE_FORWARD_Y = 0.0; LOADING_ZONE_FORWARD_H = 0.0;
        GATE_FORWARD_X = 0.0; GATE_FOWARD_Y = 0.0; GATE_FOWARD_H = 0.0;

        GATE_X = -63.975; GATE_Y = -40.256; GATE_H = 0;
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
    protected Pose getShootingPose() { return new Pose(SHOOT_START_X, SHOOT_START_Y, SHOOT_START_H); }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    @Override
    protected Pose getLoadingZoneIntakePose(){return new Pose(LOADING_ZONE_INTAKE_X, LOADING_ZONE_INTAKE_Y, LOADING_ZONE_INTAKE_H);};

    @Override
    protected Pose getGateIntakePose(){return new Pose(GATE_INTAKE_X, GATE_INTAKE_Y, GATE_INTAKE_H);};

    @Override
    protected Pose getLoadingZoneIntakeForwardPose(){return new Pose(LOADING_ZONE_FORWARD_X, LOADING_ZONE_FORWARD_Y, LOADING_ZONE_FORWARD_H);};
    @Override
    protected Pose getGateIntakeForwardPose(){return new Pose(GATE_FORWARD_X, GATE_FOWARD_Y, GATE_FOWARD_H);};
}
