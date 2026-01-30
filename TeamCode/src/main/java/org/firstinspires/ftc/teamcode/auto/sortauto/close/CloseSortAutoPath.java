package org.firstinspires.ftc.teamcode.auto.sortauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
public abstract class CloseSortAutoPath extends CloseSortAuto {
    public static double
            SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H,
            INTAKE_FORWARD_1_X, INTAKE_FORWARD_1_Y, INTAKE_FORWARD_1_H,
            SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H,
            SHOOT_2_SPIN_X, SHOOT_2_SPIN_Y, SHOOT_2_SPIN_H,
            SHOOT_2_GUARANTEE_X, SHOOT_2_GUARANTEE_Y, SHOOT_2_GUARANTEE_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H,
            INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y, INTAKE_FORWARD_2_H,
            SHOOT_3_X, SHOOT_3_Y, SHOOT_3_H,
            SHOOT_3_SPIN_X, SHOOT_3_SPIN_Y, SHOOT_3_SPIN_H,
            SHOOT_3_GUARANTEE_X, SHOOT_3_GUARANTEE_Y, SHOOT_3_GUARANTEE_H,
            INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H,
            INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y, INTAKE_FORWARD_3_H,
            SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H,
            SHOOT_4_SPIN_X, SHOOT_4_SPIN_Y, SHOOT_4_SPIN_H,
            SHOOT_4_GUARANTEE_X, SHOOT_4_GUARANTEE_Y, SHOOT_4_GUARANTEE_H,
            GATE_1_X, GATE_1_Y, GATE_1_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_1_X = 30; SHOOT_1_Y = 30.7; SHOOT_1_H = -Math.PI/2;

        INTAKE_FORWARD_1_X = 30.7; INTAKE_FORWARD_1_Y = 10.5; INTAKE_FORWARD_1_H = -Math.PI/2;
        GATE_1_X = 38; GATE_1_Y = -3; GATE_1_H = Math.PI;
        SHOOT_2_X = -2.1; SHOOT_2_Y = 8.6; SHOOT_2_H = Math.PI;
        SHOOT_2_SPIN_X = -4.1; SHOOT_2_SPIN_Y = 8.5; SHOOT_2_SPIN_H = -2.28;
        SHOOT_2_GUARANTEE_X = -1.4; SHOOT_2_GUARANTEE_Y = 15.5; SHOOT_2_GUARANTEE_H = -2.87;

        INTAKE_2_X = 57; INTAKE_2_Y = 33; INTAKE_2_H = -2.87;
        INTAKE_FORWARD_2_X = 57.5; INTAKE_FORWARD_2_Y = 12; INTAKE_FORWARD_2_H = -Math.PI/2;
        SHOOT_3_X = -2.1; SHOOT_3_Y = 8.6; SHOOT_3_H = Math.PI;
        SHOOT_3_SPIN_X = -2; SHOOT_3_SPIN_Y = 9; SHOOT_3_SPIN_H = -2.28;
        SHOOT_3_GUARANTEE_X = 2.2; SHOOT_3_GUARANTEE_Y = 15.8; SHOOT_3_GUARANTEE_H = -2.94;

        INTAKE_3_X = 83; INTAKE_3_Y = 33; INTAKE_3_H = -2.9;
        INTAKE_FORWARD_3_X = 84; INTAKE_FORWARD_3_Y = 13; INTAKE_FORWARD_3_H = -Math.PI/2;
        SHOOT_4_X = -2.1; SHOOT_4_Y = 14; SHOOT_4_H = Math.PI;
        SHOOT_4_SPIN_X = -3.4; SHOOT_4_SPIN_Y = 12.8; SHOOT_4_SPIN_H = -2.29;
        SHOOT_4_GUARANTEE_X = -7.0; SHOOT_4_GUARANTEE_Y = 24.44; SHOOT_4_GUARANTEE_H = -2.804;

        PARK_X = -7.0; PARK_Y = 30; PARK_H = -2.804;
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
    protected Pose getShoot1Pose() {
        return new Pose(SHOOT_1_X, SHOOT_1_Y*getSign(), SHOOT_1_H*getSign());
    }

    @Override
    protected Pose getShoot2Pose() {
        return new Pose(SHOOT_2_X, SHOOT_2_Y*getSign(), SHOOT_2_H*getSign());
    }
    @Override
    protected Pose getShoot2SpinPose() {
        return new Pose(SHOOT_2_SPIN_X, SHOOT_2_SPIN_Y*getSign(), SHOOT_2_SPIN_H*getSign());
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
    protected Pose getShoot3SpinPose() {
        return new Pose(SHOOT_3_SPIN_X, SHOOT_3_SPIN_Y*getSign(), SHOOT_3_SPIN_H*getSign());
    }
    @Override
    protected Pose getShoot4GuaranteePose() {
        return new Pose(SHOOT_4_GUARANTEE_X, SHOOT_4_GUARANTEE_Y*getSign(), SHOOT_4_GUARANTEE_H*getSign());
    }
    @Override
    protected Pose getShoot2GuaranteePose() {
        return new Pose(SHOOT_2_GUARANTEE_X, SHOOT_2_GUARANTEE_Y*getSign(), SHOOT_2_GUARANTEE_H*getSign());
    }
    @Override
    protected Pose getShoot3GuaranteePose() {
        return new Pose(SHOOT_3_GUARANTEE_X, SHOOT_3_GUARANTEE_Y*getSign(), SHOOT_3_GUARANTEE_H*getSign());
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
    protected Pose getShoot4SpinPose() {
        return new Pose(SHOOT_4_SPIN_X, SHOOT_4_SPIN_Y*getSign(), SHOOT_4_SPIN_H*getSign());
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
    protected Pose getGate1Pose() {return new Pose(GATE_1_X, GATE_1_Y*getSign(), GATE_1_H*getSign());}

}
