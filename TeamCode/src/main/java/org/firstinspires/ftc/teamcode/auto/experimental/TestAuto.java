package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Config
@Autonomous(name = "Test Auto", group = "Test")
public class TestAuto extends TestAutoPath {

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
            INTAKE_4_X, INTAKE_4_Y, INTAKE_4_H,
            INTAKE_FORWARD_4_X, INTAKE_FORWARD_4_Y, INTAKE_FORWARD_4_H,
            SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H,
            GATE_X, GATE_Y, GATE_H,
            PARK_X, PARK_Y, PARK_H;
    public static Pose testPose = new Pose(0, 0, Math.toRadians(0));

    static {
        /*home pedro testing:
        SHOOT_1_X = -47.654; SHOOT_1_Y = -54.089; SHOOT_1_H = Math.PI/2;
        SHOOT_2_X = -65.958; SHOOT_2_Y = 24.031; SHOOT_2_H = 0;
        SHOOT_3_X = 1.126; SHOOT_3_Y = 0.217; SHOOT_3_H = -Math.PI;
         */
        /* far auto
        SHOOT_1_X = 7.67; SHOOT_1_Y = -3.46; SHOOT_1_H = 0.40486;
        INTAKE_1_X = 27.71; INTAKE_1_Y = 12.5; INTAKE_1_H = Math.toRadians(90);
        SHOOT_2_X = 7.67; SHOOT_2_Y = -3.46; SHOOT_2_H = 0.40486;
        INTAKE_2_X = 50.94; INTAKE_2_Y = 12.5; INTAKE_2_H = Math.toRadians(90);
        SHOOT_3_X = 7.67; SHOOT_3_Y = -3.46; SHOOT_3_H = 0.40486;
        INTAKE_3_X = 74.58; INTAKE_3_Y = 12.5; INTAKE_3_H = Math.toRadians(90);
        SHOOT_4_X = 7.67; SHOOT_4_Y = -3.46; SHOOT_4_H = 0.40486;
         */
        // close auto: shoot pos is 58 inches from goal

        SHOOT_1_X = 8.493; SHOOT_1_Y = 0.686; SHOOT_1_H = 0.390;
        SHOOT_2_X = 8.493; SHOOT_2_Y = 0.686; SHOOT_2_H = 0.390;
        SHOOT_3_X = 8.493; SHOOT_3_Y = 0.686; SHOOT_3_H = 0.390;

//        SHOOT_1_X = 47.743; SHOOT_1_Y = -1.066; SHOOT_1_H = -2.35;
        INTAKE_1_X = 27.893; INTAKE_1_Y = 14.743; INTAKE_1_H = Math.PI/2;
        INTAKE_FORWARD_1_X = 27.893; INTAKE_FORWARD_1_Y = 32.840; INTAKE_FORWARD_1_H = Math.PI/2;
//        SHOOT_2_X = 45.743; SHOOT_2_Y = -3.066; SHOOT_2_H = -2.35;
        INTAKE_2_X = 50.394; INTAKE_2_Y = 13.582; INTAKE_2_H = Math.PI/2;
        INTAKE_FORWARD_2_X = 50.394; INTAKE_FORWARD_2_Y = 32.220; INTAKE_FORWARD_2_H = Math.PI/2;
//        SHOOT_3_X = 45.743; SHOOT_3_Y = -3.066; SHOOT_3_H = -2.35;
        INTAKE_3_X = 74.885; INTAKE_3_Y = 13.874; INTAKE_3_H = Math.PI/2;
        INTAKE_FORWARD_3_X = 74.885; INTAKE_FORWARD_3_Y = 31.516; INTAKE_FORWARD_3_H = Math.PI/2;

        INTAKE_4_X = 5; INTAKE_4_Y = 5; INTAKE_4_H = 5;
        INTAKE_FORWARD_4_X = 5; INTAKE_FORWARD_4_Y = 5; INTAKE_FORWARD_4_H = 5;
        SHOOT_4_X = 8.493; SHOOT_4_Y = 0.686; SHOOT_4_H = 0.390;
        GATE_X = 56.620; GATE_Y = 37.334; GATE_H = Math.PI;
        PARK_X = 25.113; PARK_Y = 5.868; PARK_H = 0.390;
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.toRadians(180));
    }

    @Override
    protected boolean isRed() {
        return false;
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
    protected Pose getIntake4Pose() {
        return new Pose(INTAKE_4_X, INTAKE_4_Y*getSign(), INTAKE_4_H*getSign());
    }

    @Override
    protected Pose getParkPose() {
        return new Pose(PARK_X, PARK_Y*getSign(), PARK_H*getSign());
    }

    @Override
    protected Pose getIntake1ForwardPose() {return new Pose(INTAKE_FORWARD_1_X,  INTAKE_FORWARD_1_Y*getSign(), INTAKE_FORWARD_1_H*getSign());}

    @Override
    protected Pose getIntake2ForwardPose() {return new Pose(INTAKE_FORWARD_2_X, INTAKE_FORWARD_2_Y*getSign(), INTAKE_FORWARD_2_H*getSign());}

    @Override
    protected Pose getIntake3ForwardPose() {return new Pose(INTAKE_FORWARD_3_X, INTAKE_FORWARD_3_Y*getSign(), INTAKE_FORWARD_3_H*getSign());}

    @Override
    protected Pose getIntake4ForwardPose() {return new Pose(INTAKE_FORWARD_4_X, INTAKE_FORWARD_4_Y*getSign(), INTAKE_FORWARD_4_H*getSign());}

    @Override
    protected Pose getGatePose() {return new Pose(GATE_X, GATE_Y*getSign(), GATE_H*getSign());}
}

