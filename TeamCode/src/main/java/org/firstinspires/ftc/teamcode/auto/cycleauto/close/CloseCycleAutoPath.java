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
            INTAKE_GATE_FORWARD_X, INTAKE_GATE_FORWARD_Y, INTAKE_GATE_FORWARD_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H,
            INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y, INTAKE_FORWARD_2_H,
            INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H,
            INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y, INTAKE_FORWARD_3_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_X = 27.631; SHOOT_Y = 40.047; SHOOT_H = -0.76;

        INTAKE_1_X = 55.142; INTAKE_1_Y = 27.694; INTAKE_1_H = -Math.PI/2;
        INTAKE_FORWARD_1_X = 54.370; INTAKE_FORWARD_1_Y = 7.728; INTAKE_FORWARD_1_H = -Math.PI/2;

        INTAKE_GATE_X = 52.094; INTAKE_GATE_Y = -4; INTAKE_GATE_H = -1.891;
        INTAKE_GATE_FORWARD_X = 60; INTAKE_GATE_FORWARD_Y = -4.5; INTAKE_GATE_FORWARD_H = -Math.PI*2/3;

        INTAKE_2_X = 78.833; INTAKE_2_Y = 27.553; INTAKE_2_H = -Math.PI/2;
        INTAKE_FORWARD_2_X = 78.277; INTAKE_FORWARD_2_Y = 8.560; INTAKE_FORWARD_2_H = -Math.PI/2;

        INTAKE_3_X = 30.652; INTAKE_3_Y = 27.387; INTAKE_3_H = -Math.PI/2;
        INTAKE_FORWARD_3_X = 31.126; INTAKE_FORWARD_3_Y = 10.867; INTAKE_FORWARD_3_H = -Math.PI/2;

        PARK_X = 47.631; PARK_Y = 20.047; PARK_H = -2.27;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0*getSign(),-Math.PI*getSign());
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
    protected Pose getIntake3Pose() {
        return new Pose(INTAKE_3_X, INTAKE_3_Y*getSign(), INTAKE_3_H*getSign());
    }



    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    protected Pose getIntake1ForwardPose() {return new Pose(INTAKE_FORWARD_1_X,  INTAKE_FORWARD_1_Y*getSign(), INTAKE_FORWARD_1_H*getSign());}

    @Override
    protected Pose getIntake2ForwardPose() {return new Pose(INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y*getSign(), INTAKE_FORWARD_2_H*getSign());}

    @Override
    protected Pose getIntake3ForwardPose() {return new Pose(INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y*getSign(), INTAKE_FORWARD_3_H*getSign());}

    @Override
    protected Pose getShootPose() {
        return new Pose(SHOOT_X, SHOOT_Y*getSign(), SHOOT_H*getSign());
    }

    @Override
    protected Pose getIntakeGatePose() {
        return new Pose(INTAKE_GATE_X, INTAKE_GATE_Y*getSign(), INTAKE_GATE_H*getSign());
    }

    @Override
    protected Pose getIntakeGateForwardPose() {
        return new Pose(INTAKE_GATE_FORWARD_X, INTAKE_GATE_FORWARD_Y*getSign(), INTAKE_GATE_FORWARD_H*getSign());
    }


}
