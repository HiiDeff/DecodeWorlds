package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

@Config
public class Turret extends PIDModel {

    public static double LIMELIGHT_DIST_TO_TURRET_CENTER_INCH = 5.25;
    public static double TURRET_DIST_TO_ROBOT_CENTER_INCH = 3.75;
    private final RobotBase robot;
    private final double ticksPerRadian;
    private int targetAngle;
    private double prevLimelightHeading = 0.0;
    private ElapsedTime timer = null;
    public static double THRESHOLD_RAD_PER_SEC = 0.3, THRESHOLD_ROBOT_VELOCITY = 10; //tuned
    public Turret(RobotBase robot, PIDCoefficients pidCoefficients, double ticksPerRadian) {
        super(pidCoefficients);
        this.robot = robot;
        this.ticksPerRadian = ticksPerRadian;
    }

    public Pose calcRobotPose(Pose limelightPose) {
        double turretAngleRad = robot.getTurretAngleTicks()/ticksPerRadian*Math.PI;
        double robotHeading = robot.getHeading();
        double limelightHeading = Utils.normalize(robotHeading+turretAngleRad);

        if(timer == null || timer.seconds() >= 1) { //haven't used calcRobotPose in a while
            timer = new ElapsedTime();
            prevLimelightHeading = limelightHeading;
            return null;
        } else {
            double dTheta_dT = (limelightHeading-prevLimelightHeading)/timer.seconds();
            Log.i("edbug turret dTheta_dT", dTheta_dT+"");
            prevLimelightHeading = limelightHeading;
            timer.reset();
            if(Math.abs(dTheta_dT)>THRESHOLD_RAD_PER_SEC) return null;
        }
        Log.i("edbug robot velocity", robot.getVelocity().getMagnitude()+"");
        if(robot.getVelocity().getMagnitude()>THRESHOLD_ROBOT_VELOCITY) return null;

        Vector toTurretCenter = new Vector(-LIMELIGHT_DIST_TO_TURRET_CENTER_INCH, limelightHeading);
        Pose turretCenter = limelightPose.plus(new Pose(toTurretCenter.getXComponent(), toTurretCenter.getYComponent()));
        Vector toRobotCenter = new Vector(TURRET_DIST_TO_ROBOT_CENTER_INCH, robotHeading);
        Pose robotPose = turretCenter.plus(new Pose(toRobotCenter.getXComponent(), toRobotCenter.getYComponent(), robotHeading));

        Log.i("edbug vals", turretAngleRad+" "+robotHeading+" "+limelightHeading);
        Log.i("ndbug turret center vector", toTurretCenter.getXComponent() + " " + toTurretCenter.getYComponent());
        Log.i("ndbug turrent center pose", turretCenter.getX() + " " + turretCenter.getY() + " " + turretCenter.getHeading());
        Log.i("ndbug robot center vector", toRobotCenter.getXComponent() + " " + toRobotCenter.getYComponent());
        Log.i("ndbug robot center pose", robotPose.getX() + " " + robotPose.getY() + " " + robotPose.getHeading());

        return robotPose.setHeading(robotHeading);
    }

    public double calcTurretAngleToGoal() {
        double robotAngleToGoal = robot.limelightAprilTagDetector.getAngleToGoal(robot.getPose());

        return 0.0;
    }
    public double calcTurretAngleToGoalWithVelocity() {



        return 0.0;
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
