package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

public class Turret extends PIDModel {

    public static double LIMELIGHT_DIST_TO_TURRET_CENTER_INCH = 6.0;
    public static double TURRET_DIST_TO_ROBOT_CENTER_INCH = 2.75;
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
        Vector toTurretCenter = limelightPose.getHeadingAsUnitVector().times(-LIMELIGHT_DIST_TO_TURRET_CENTER_INCH);
        Pose turretCenter = limelightPose.plus(new Pose(toTurretCenter.getXComponent(), toTurretCenter.getYComponent()));
        double robotHeading = limelightPose.getHeading()-turretAngleRad;
        Vector toRobotCenter = new Vector(TURRET_DIST_TO_ROBOT_CENTER_INCH, robotHeading);
        Pose robotPose = turretCenter.plus(new Pose(toRobotCenter.getXComponent(), toRobotCenter.getYComponent(), robotHeading));
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
