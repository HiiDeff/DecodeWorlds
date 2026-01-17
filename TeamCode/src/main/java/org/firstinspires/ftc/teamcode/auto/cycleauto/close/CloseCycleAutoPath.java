package org.firstinspires.ftc.teamcode.auto.cycleauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class CloseCycleAutoPath extends CloseCycleAuto {
    public static double
            SHOOT_X, SHOOT_Y, SHOOT_H,
            INTAKE_1_X, INTAKE_1_Y, INTAKE_1_H,
            INTAKE_FORWARD_1_X, INTAKE_FORWARD_1_Y, INTAKE_FORWARD_1_H,
            INTAKE_GATE_X, INTAKE_GATE_Y, INTAKE_GATE_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H,
            INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y, INTAKE_FORWARD_2_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_X = 47.913; SHOOT_Y = 22.202; SHOOT_H = -2.39;

        INTAKE_1_X = 51.835; INTAKE_1_Y = 15; INTAKE_1_H = -Math.PI/2;
        INTAKE_FORWARD_1_X = 51.835; INTAKE_FORWARD_1_Y = -4; INTAKE_FORWARD_1_H = -Math.PI/2;
        INTAKE_GATE_X = 58.933; INTAKE_GATE_Y = -18; INTAKE_GATE_H = 3.0;

        INTAKE_2_X = 71; INTAKE_2_Y = 10.706; INTAKE_2_H = -Math.PI/2;
        INTAKE_FORWARD_2_X = 75.191; INTAKE_FORWARD_2_Y = -12.704; INTAKE_FORWARD_2_H = -Math.PI/2;

        PARK_X = 67.913; PARK_Y = 2.202; PARK_H = -2.39;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(3,-3*getSign(),-Math.PI/2*getSign());
    }

    @Override
    protected boolean isFar() {
        return false;
    }

    @Override
    protected Pose getIntake1Pose() {
        return new Pose(INTAKE_1_X, INTAKE_1_Y*getSign(), INTAKE_1_H*getSign());
    }

    @Override
    protected Pose getIntake2Pose() {
        return new Pose(INTAKE_2_X, INTAKE_2_Y*getSign(), INTAKE_2_H*getSign());
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    protected Pose getIntake1ForwardPose() {return new Pose(INTAKE_FORWARD_1_X,  INTAKE_FORWARD_1_Y*getSign(), INTAKE_FORWARD_1_H*getSign());}

    @Override
    protected Pose getIntake2ForwardPose() {return new Pose(INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y*getSign(), INTAKE_FORWARD_2_H*getSign());}

    @Override
    protected Pose getShootPose() {
        return new Pose(SHOOT_X, SHOOT_Y*getSign(), SHOOT_H*getSign());
    }

    @Override
    protected Pose getIntakeGatePose() {
        return new Pose(INTAKE_GATE_X, INTAKE_GATE_Y*getSign(), INTAKE_GATE_H*getSign());
    }

}
