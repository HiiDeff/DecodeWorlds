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
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_START_X = -11.778; SHOOT_START_Y = 4; SHOOT_START_H = -2.75;

        LOADING_ZONE_INTAKE_X = -17.107; LOADING_ZONE_INTAKE_Y = -40; LOADING_ZONE_INTAKE_H = -0.547;
        LOADING_ZONE_FORWARD_X = -4.334; LOADING_ZONE_FORWARD_Y = -41; LOADING_ZONE_FORWARD_H = -1.260;
        GATE_INTAKE_X = -37.107; GATE_INTAKE_Y = -40; GATE_INTAKE_H = -0.547;
        GATE_FORWARD_X = -24.334; GATE_FOWARD_Y = -41; GATE_FOWARD_H = -1.260;

        PARK_X = -4.980; PARK_Y = -1.086; PARK_H = -1.736;
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
    protected Pose getShootingPose() { return new Pose(SHOOT_START_X, SHOOT_START_Y*getSign(), SHOOT_START_H*getSign()); }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    @Override
    protected Pose getLoadingZoneIntakePose(){return new Pose(LOADING_ZONE_INTAKE_X, LOADING_ZONE_INTAKE_Y*getSign(), LOADING_ZONE_INTAKE_H*getSign());}

    @Override
    protected Pose getGateIntakePose(){return new Pose(GATE_INTAKE_X, GATE_INTAKE_Y*getSign(), GATE_INTAKE_H*getSign());}
    @Override
    protected Pose getLoadingZoneIntakeForwardPose(){return new Pose(LOADING_ZONE_FORWARD_X, LOADING_ZONE_FORWARD_Y*getSign(), LOADING_ZONE_FORWARD_H*getSign());}
    @Override
    protected Pose getGateIntakeForwardPose(){return new Pose(GATE_FORWARD_X, GATE_FOWARD_Y*getSign(), GATE_FOWARD_H*getSign());}
}
