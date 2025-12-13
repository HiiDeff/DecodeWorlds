package org.firstinspires.ftc.teamcode.pedropathing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.filter.MovingAverage;

import java.util.List;

@Config
public abstract class MecanumDrive extends Follower {

    // Constants
    private Pose defaulStartPose = new Pose();
    public static double LATERAL_MULTIPLIER = 1.0; // adjust if robot is skewed
    public static double TRACK_WIDTH = 1.0; // adjust for robot geometry

    // Pedro
    private static Mecanum mecanum;
    private List<DcMotorEx> motors; //leftFront, leftRear, rightFront, rightRear;

    // Angular Velocity & Acceleration
    private ElapsedTime timer = null;
    private final MovingAverage angularVelocityFilter = new MovingAverage(5);
    private final MovingAverage angularAccelerationFilter = new MovingAverage(5);
    private double lastHeading = 0.0, lastFilteredAngularVelocity = 0.0;
    private final MovingAverage velXFilter = new MovingAverage(5);
    private final MovingAverage velYFilter = new MovingAverage(5);
    private final MovingAverage accelXFilter = new MovingAverage(5);
    private final MovingAverage accelYFilter = new MovingAverage(5);
    private Pose lastPose = new Pose();
    private double lastFilteredVx = 0.0, lastFilteredVy = 0.0;

    //Translational Acceleration

    //Transfer Values Between Auto and TeleOp
    public static double heading = 0;

    public MecanumDrive(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, PinpointLocalizer localizer, PathConstraints pathConstraints) {
        super(followerConstants, localizer, createMecanum(hardwareMap, driveConstants), pathConstraints);
        motors = mecanum.getMotors();
    }

    private static Mecanum createMecanum(HardwareMap hardwareMap, MecanumConstants driveConstants) {
        mecanum = new Mecanum(hardwareMap, driveConstants);
        return mecanum;
    }

    public void init(Pose startPose) {
        setStartingPose(startPose);
    }

    @Override
    public void update() {
        super.update();
        updateHeading();
        updateAcceleration();
    }
    public void updateHeading() {
        heading = getHeading();
        Log.i("edbug raw heading", heading+"");
    }
    public void updateHeading(double override) {
        heading = override;
        Log.i("edbug raw heading", heading+"");
    }
    private void updateAcceleration() {
        Pose pose = getPose();

        if (timer == null) {
            timer = new ElapsedTime();
            lastHeading = heading;
            lastFilteredAngularVelocity = 0.0;
            lastPose = new Pose();
            lastFilteredVx = 0.0;
            lastFilteredVy = 0.0;
            return;
        }
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 1e-6 || dt > 1) return;

        // Angular Velocity

        double deltaHeading = Utils.normalize(heading - lastHeading);
        lastHeading = heading;

        angularVelocityFilter.add(deltaHeading / dt);
        double filteredAngularVelocity = angularVelocityFilter.get();

        double angularAcceleration = (filteredAngularVelocity - lastFilteredAngularVelocity) / dt;
        lastFilteredAngularVelocity = filteredAngularVelocity;

        angularAccelerationFilter.add(angularAcceleration);

        // Translational Velocity
        Pose deltaPose = pose.copy().minus(lastPose);
        lastPose = pose;

        velXFilter.add(deltaPose.getX()/dt);
        velYFilter.add(deltaPose.getY()/dt);
        double filteredVx = velXFilter.get();
        double filteredVy = velYFilter.get();

        double ax = (filteredVx - lastFilteredVx) / dt;
        double ay = (filteredVy - lastFilteredVy) / dt;
        lastFilteredVx = filteredVx;
        lastFilteredVy = filteredVy;

        accelXFilter.add(ax);
        accelYFilter.add(ay);

        Log.i("edbug angular accel", angularAccelerationFilter.get()+"");
        Log.i("edbug translational velo", "("+velXFilter.get()+" "+velYFilter.get()+")");
        Log.i("edbug translational accel", "("+accelXFilter.get()+" "+accelYFilter.get()+")");

        // Heading for transfer between teleop and auto
    }

    public Vector getTranslationalVelocity() {
        return new Vector(new Pose(velXFilter.get(), velYFilter.get()));
    }

    public Vector getTranslationalAcceleration() {
        return new Vector(new Pose(accelXFilter.get(), accelYFilter.get()));
    }

    public double getAngularAcceleration() {
        return angularAccelerationFilter.get();
    }


    public void setDrivePowers(double x, double y, double a) {
        // Mecanum Kinematics
        double leftFrontPower  = x - y * LATERAL_MULTIPLIER - a * TRACK_WIDTH;
        double leftBackPower   = x + y * LATERAL_MULTIPLIER - a * TRACK_WIDTH;
        double rightBackPower  = x - y * LATERAL_MULTIPLIER + a * TRACK_WIDTH;
        double rightFrontPower = x + y * LATERAL_MULTIPLIER + a * TRACK_WIDTH;

        // Normalize powers
        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightBackPower), Math.abs(rightFrontPower))
        );
        if (max < 1.0) max = 1.0;

        // Set Powers
        motors.get(0).setPower(leftFrontPower / max);
        motors.get(1).setPower(leftBackPower / max);
        motors.get(2).setPower(rightFrontPower/ max);
        motors.get(3).setPower(rightBackPower / max);
    }
}
