package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.filter.MovingAverage;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

@Config
public class Turret extends PIDModel {

    // all distances below are extensively tuned & correct
    public static double LIMELIGHT_DIST_TO_TURRET_CENTER_INCH = 5.5;
    public static double SHOOTER_COM_MAGNITUDE = 10.0; //technically wrong, but well tuned
    public static double TURRET_X_DIST_TO_ROBOT_CENTER_INCH = 3.25; //forwards is positive 4.0?
    public static double TURRET_Y_DIST_TO_ROBOT_CENTER_INCH = -0.75; //left is positive
    private final RobotBase robot;
    private final double ticksPerRadian;
    public static boolean velocityFeedForwardActive = false;
    private int targetAngle;
    private double prevLimelightHeading = 0.0;
    private ElapsedTime headingTimer = null, targetTimer = null;
    public static double THRESHOLD_RAD_PER_SEC = 0.3, THRESHOLD_ROBOT_VELOCITY = 10; //tuned
    private double limelightHeading = 0.0;
    private double prevRobotAngleToGoal = 0.0;
    private MovingAverage dTarget_dT;
    private Pose turretCenter = new Pose(0, 0);

    public Turret(RobotBase robot, PIDCoefficients pidCoefficients, double ticksPerRadian) {
        super(pidCoefficients);
        this.robot = robot;
        this.ticksPerRadian = ticksPerRadian;
    }

    public Pose calcRobotPose(Pose limelightPose) {
        double turretAngleRad = robot.getTurretAngleTicks()/ticksPerRadian;
        double robotHeading = robot.getHeading();
        limelightHeading = Utils.normalize(robotHeading+turretAngleRad);
        Log.i("limelightPose", limelightPose+"");

        if(headingTimer == null || headingTimer.seconds() >= 1) { //haven't used calcRobotPose in a while
            headingTimer = new ElapsedTime();
            prevLimelightHeading = limelightHeading;
            return null;
        } else {
            double dTheta_dT = Utils.normalize(limelightHeading-prevLimelightHeading)/ headingTimer.seconds(); //TODO: check jumps between -PI <-> PI
            Log.i("edbug turret dTheta_dT", dTheta_dT+"");
            prevLimelightHeading = limelightHeading;
            headingTimer.reset();
            if(Math.abs(dTheta_dT)>THRESHOLD_RAD_PER_SEC) return null;
        }
        Log.i("edbug robot velocity", robot.getVelocity().getMagnitude()+"");
        if(robot.getVelocity().getMagnitude()>THRESHOLD_ROBOT_VELOCITY) return null;

        Vector toTurretCenter = new Vector(-LIMELIGHT_DIST_TO_TURRET_CENTER_INCH, limelightHeading);
        turretCenter = limelightPose.plus(new Pose(toTurretCenter.getXComponent(), toTurretCenter.getYComponent()));

        Vector toRobotCenterX = new Vector(TURRET_X_DIST_TO_ROBOT_CENTER_INCH, robotHeading);
        Vector toRobotCenterY = new Vector(TURRET_Y_DIST_TO_ROBOT_CENTER_INCH, robotHeading+Math.PI/2);
        Vector toRobotCenter = toRobotCenterX.plus(toRobotCenterY);

        Pose robotPose = turretCenter.copy().plus(new Pose(toRobotCenter.getXComponent(), toRobotCenter.getYComponent()));

        Log.i("edbug vals", turretAngleRad+" "+robotHeading+" "+limelightHeading);
        Log.i("ndbug turret center vector", toTurretCenter.getXComponent() + " " + toTurretCenter.getYComponent());
        Log.i("ndbug turrent center pose", turretCenter.getX() + " " + turretCenter.getY() + " " + turretCenter.getHeading());
        Log.i("ndbug robot centerX vector", toRobotCenterX.getXComponent() + " " + toRobotCenterX.getYComponent());
        Log.i("ndbug robot center vector", toRobotCenterY.getXComponent() + " " + toRobotCenterY.getYComponent());
        Log.i("ndbug robot center pose", robotPose.getX() + " " + robotPose.getY() + " " + robotPose.getHeading());

        return turretCenter.setHeading(robotHeading);
    }
    public Pose getTurretCenter() {
        return turretCenter;
    }

    // PID Control
    public void setTargetAngle(int targetAngleTicks) {
        Log.i("targetAngle", targetAngleTicks+"");
        if(targetTimer == null || targetTimer.seconds() >= 1) {
            dTarget_dT = new MovingAverage(3);
            targetTimer = new ElapsedTime();
        } else {
            dTarget_dT.add((targetAngleTicks-prevRobotAngleToGoal)/(targetTimer.milliseconds()));
            targetTimer.reset();
        }
        prevRobotAngleToGoal = targetAngleTicks;
        targetAngle = targetAngleTicks;
    }

    public double getTurretAngle() {
        return robot.getTurretAngleTicks()/ticksPerRadian;
    }

    @Override
    public double getError() {
        int error = targetAngle - robot.getTurretAngleTicks();
        return error;
        //return Math.sqrt(Math.abs(error))*Math.copySign(1, error);
    }

    @Override
    public double getFeedForward() {
        Vector r = new Vector(SHOOTER_COM_MAGNITUDE, limelightHeading); //shooter COM 2.75 according to clevin
        Vector a = robot.getTranslationalAcceleration(); //robot translational accel
        double alpha = robot.getAngularAcceleration(); //robot angular accel

        double tau_trans = (r.getXComponent() * a.getYComponent() - r.getYComponent() * a.getXComponent());
        double tau_ang = alpha * (Math.pow(r.getMagnitude(), 2));
        double tau_ff = tau_trans + tau_ang;

        Log.i("edbug trans accel", a+"");
        Log.i("edbug tau_ff", tau_ff+"");

        return tau_ff;
    }

    public void activateVelocityFeedForward() {
        velocityFeedForwardActive = true;
    }
    public void disableVelocityFeedForward() {
        velocityFeedForwardActive = false;
    }

    @Override
    public double getFeedForward2() {
        if(dTarget_dT==null || !velocityFeedForwardActive) return 0;
        Log.i("edbug dTarget_dT", dTarget_dT.get()+"");
        return dTarget_dT.get();
    }

    @Override
    protected double getStopError() {
        return 10;
        //return 0; //don't stop
    }

    @Override
    protected double getStopErrorDerivative() {
        return 0;
    }

    @Override
    public void cancel() {
        //do nothing
    }
}
