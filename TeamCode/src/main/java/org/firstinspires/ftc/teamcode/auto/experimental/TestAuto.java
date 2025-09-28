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
    
    public static double SHOOT_1_X = 7.67, SHOOT_1_Y = -3.46, SHOOT_1_H = 0.40486;
    public static double INTAKE_1_X = 27.71, INTAKE_1_Y = 12.5, INTAKE_1_H = Math.toRadians(90);
    public static double SHOOT_2_X = 7.67, SHOOT_2_Y = -3.46, SHOOT_2_H = 0.40486;
    public static double INTAKE_2_X = 50.94, INTAKE_2_Y = 12.5, INTAKE_2_H = Math.toRadians(90);
    public static double SHOOT_3_X = 7.67, SHOOT_3_Y = -3.46, SHOOT_3_H = 0.40486;
    public static double INTAKE_3_X = 74.58, INTAKE_3_Y = 12.5, INTAKE_3_H = Math.toRadians(90);
    public static double SHOOT_4_X = 7.67, SHOOT_4_Y = -3.46, SHOOT_4_H = 0.40486;

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.toRadians(0));
    }

    @Override
    protected boolean isRed() {
        return false;
    }

    @Override
    protected Pose getShoot1Pose() {
        return new Pose(SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H);
    }

    @Override
    protected Pose getIntake1Pose() {
        return new Pose(INTAKE_1_X, INTAKE_1_Y, INTAKE_1_H);
    }

    @Override
    protected Pose getShoot2Pose() {
        return new Pose(SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H);
    }

    @Override
    protected Pose getIntake2Pose() {
        return new Pose(INTAKE_2_X, INTAKE_2_Y, INTAKE_2_H);
    }

    @Override
    protected Pose getShoot3Pose() {
        return new Pose(SHOOT_3_X, SHOOT_3_Y, SHOOT_3_H);
    }

    @Override
    protected Pose getIntake3Pose() {
        return new Pose(INTAKE_3_X, INTAKE_3_Y, INTAKE_3_H);
    }

    @Override
    protected Pose getShoot4Pose() {
        return new Pose(SHOOT_4_X, SHOOT_4_Y, SHOOT_4_H);
    }
}
