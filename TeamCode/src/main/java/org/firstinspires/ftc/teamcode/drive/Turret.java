package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

@Config
public class Turret extends PIDModel {

    public static double LIMELIGHT_DIST_TO_TURRET_CENTER_INCH = 5.0;
    public static double TURRET_DIST_TO_ROBOT_CENTER_INCH = 3.5;
    private final RobotBase robot;
    private final double ticksPerRadian;
    private int targetAngle;
    public Turret(RobotBase robot, PIDCoefficients pidCoefficients, double ticksPerRadian) {
        super(pidCoefficients);
        this.robot = robot;
        this.ticksPerRadian = ticksPerRadian;
    }

    public Pose calcRobotPose(Pose limelightPose) {
        double turretAngleRad = robot.getTurretAngleTicks()/ticksPerRadian;
        double robotHeading = robot.getIMUHeading();
        double limelightHeading = Utils.normalize(robotHeading+turretAngleRad);
        Log.i("edbug vals", turretAngleRad+" "+robotHeading+" "+limelightHeading);
        Vector toTurretCenter = new Vector(-LIMELIGHT_DIST_TO_TURRET_CENTER_INCH, limelightHeading);
        Log.i("ndbug turret center vector", toTurretCenter.getXComponent() + " " + toTurretCenter.getYComponent());
        Pose turretCenter = limelightPose.plus(new Pose(toTurretCenter.getXComponent(), toTurretCenter.getYComponent()));
        Log.i("ndbug turrent center pose", turretCenter.getX() + " " + turretCenter.getY() + " " + turretCenter.getHeading());
        Vector toRobotCenter = new Vector(TURRET_DIST_TO_ROBOT_CENTER_INCH, robotHeading);
        Log.i("ndbug robot center vector", toRobotCenter.getXComponent() + " " + toRobotCenter.getYComponent());
        Pose robotPose = turretCenter.plus(new Pose(toRobotCenter.getXComponent(), toRobotCenter.getYComponent(), robotHeading));
        Log.i("ndbug robot center pose", robotPose.getX() + " " + robotPose.getY() + " " + robotPose.getHeading());
        return robotPose;
    }


    // PID Control
    public void setTargetAngle(int targetAngleTicks) {
        targetAngle = targetAngleTicks;
    }

    @Override
    public void cancel() {
        //do nothing
    }

    @Override
    public double getError() {
        int error = targetAngle - robot.getTurretAngleTicks();
        return error;
        //return Math.sqrt(Math.abs(error))*Math.copySign(1, error);
    }

    @Override
    protected double getStopError() {
        return 1;
        //return 0; //don't stop
    }

    @Override
    protected double getStopErrorDerivative() {
        return 0;
    }
}
