package org.firstinspires.ftc.teamcode.auto.defaultauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.Location;
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
            GATE_1_X, GATE_1_Y, GATE_1_H,
            PARK_X, PARK_Y, PARK_H;

    //meow meow meow meow meow meow meow meow meow meow meow meow meow meow meow
    //meow meow meow meow meow meow meow meow meow meow meow meow meow meow meow
    static {
        SHOOT_1_X = 41.362; SHOOT_1_Y = 19.814; SHOOT_1_H = -2.41;

        INTAKE_1_X = 48.528; INTAKE_1_Y = 13; INTAKE_1_H = -Math.PI/2;
        INTAKE_FORWARD_1_X = 48.528; INTAKE_FORWARD_1_Y = -9; INTAKE_FORWARD_1_H = -Math.PI/2;
        GATE_1_X = 54; GATE_1_Y = -15.5; GATE_1_H = Math.PI;
        SHOOT_2_X = 41.362; SHOOT_2_Y = 19.814; SHOOT_2_H = -2.41;

        INTAKE_2_X = 70.015; INTAKE_2_Y = 13; INTAKE_2_H = -Math.PI/2;
        INTAKE_FORWARD_2_X = 73.015; INTAKE_FORWARD_2_Y = -8; INTAKE_FORWARD_2_H = -Math.PI/2;
//        GATE_2_X = 65.392; GATE_2_Y = -15.622; GATE_2_H = 0;
        SHOOT_3_X = 41.362; SHOOT_3_Y = 19.814; SHOOT_3_H = -2.41;

        INTAKE_3_X = 93.885; INTAKE_3_Y = 13; INTAKE_3_H = -Math.PI/2;
        INTAKE_FORWARD_3_X = 96.885; INTAKE_FORWARD_3_Y = -8; INTAKE_FORWARD_3_H = -Math.PI/2;
        SHOOT_4_X = 41.362; SHOOT_4_Y = 19.814; SHOOT_4_H = -2.39;

        PARK_X = 61.362; PARK_Y = -1.814; PARK_H = -2.41;
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
    protected Pose getGatePose() {
        return new Pose(GATE_1_X, GATE_1_Y*getSign(), GATE_1_H*getSign());
    }

}
