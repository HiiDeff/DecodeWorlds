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
            INTAKE_1_X, INTAKE_1_Y, INTAKE_1_H,
            INTAKE_FORWARD_1_X, INTAKE_FORWARD_1_Y, INTAKE_FORWARD_1_H,
            SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H,
            INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H,
            INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y, INTAKE_FORWARD_2_H,
            SHOOT_3_X, SHOOT_3_Y, SHOOT_3_H,
            INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H,
            INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y, INTAKE_FORWARD_3_H,
            SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H,
            GATE_X, GATE_Y, GATE_H,
            PARK_X, PARK_Y, PARK_H;

    static {
        SHOOT_1_X = -42.084; SHOOT_1_Y = 14.637; SHOOT_1_H = -0.149;
        SHOOT_2_X = -42.084; SHOOT_2_Y = 14.637; SHOOT_2_H = -0.149;
        SHOOT_3_X = -42.084; SHOOT_3_Y = 14.637; SHOOT_3_H = -0.149;

        INTAKE_1_X = -56.027; INTAKE_1_Y = 44.911; INTAKE_1_H = 0.662;
        INTAKE_FORWARD_1_X = -42.217; INTAKE_FORWARD_1_Y = 56.216; INTAKE_FORWARD_1_H = 0.662;

        INTAKE_2_X = -41.743; INTAKE_2_Y = 26.299; INTAKE_2_H = 0.656;
        INTAKE_FORWARD_2_X = -27.902; INTAKE_FORWARD_2_Y = 37.528; INTAKE_FORWARD_2_H = 0.656;


        INTAKE_3_X = -69.944; INTAKE_3_Y = 64.110; INTAKE_3_H = 0.614;
        INTAKE_FORWARD_3_X = -52.197; INTAKE_FORWARD_3_Y = 78.521; INTAKE_FORWARD_3_H = 0.686;
        SHOOT_4_X = -42.084; SHOOT_4_Y = 14.637; SHOOT_4_H = -0.149;
        GATE_X = -33.681; GATE_Y = 55.217; GATE_H = 2.275;
        PARK_X = -52.456; PARK_Y = 30; PARK_H = -0.325;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,0);
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

    protected Pose getIntake1ForwardPose() {return new Pose(INTAKE_FORWARD_1_X,  INTAKE_FORWARD_1_Y*getSign(), INTAKE_FORWARD_1_H*getSign());}

    @Override
    protected Pose getIntake2ForwardPose() {return new Pose(INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y*getSign(), INTAKE_FORWARD_2_H*getSign());}

    @Override
    protected Pose getIntake3ForwardPose() {return new Pose(INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y*getSign(), INTAKE_FORWARD_3_H*getSign());}

    @Override
    protected Pose getGatePose() {return new Pose(GATE_X, GATE_Y*getSign(), GATE_H*getSign());}
}
