package org.firstinspires.ftc.teamcode.pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public abstract class MecanumDrive extends Follower {

    // Constants
    private Pose startPose = new Pose();
    public static double LATERAL_MULTIPLIER = 1.0; // adjust if robot is skewed
    public static double TRACK_WIDTH = 1.0; // adjust for robot geometry

    // Pedro
    private static Mecanum mecanum;
    private List<DcMotorEx> motors;

    public MecanumDrive(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(followerConstants, localizer, createMecanum(hardwareMap, driveConstants), pathConstraints);
        motors = mecanum.getMotors();
    }

    private static Mecanum createMecanum(HardwareMap hardwareMap, MecanumConstants driveConstants) {
        mecanum = new Mecanum(hardwareMap, driveConstants);
        return mecanum;
    }

    public void init(Pose startPose) {
        this.startPose = startPose;
        setStartingPose(startPose);
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
        motors.get(2).setPower(rightBackPower / max);
        motors.get(3).setPower(rightFrontPower / max);
    }
}
