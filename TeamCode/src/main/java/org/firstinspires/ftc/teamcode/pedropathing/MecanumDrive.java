package org.firstinspires.ftc.teamcode.pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public abstract class MecanumDrive extends Follower {

    // path constraints (pedro)
    public static double T_VALUE_CONSTRAINT = 0.99,
            TIMEOUT_CONSTRAINT = 100,
            BRAKING_STRENGTH = 1,
            BRAKING_START = 1;
    private final Pose startPose = new Pose(0,0,0.0);

    public MecanumDrive(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer) {
        super(followerConstants,
                localizer,
                new Mecanum(hardwareMap, driveConstants),
                getPathConstraints());
    }

    public void init() {
        setStartingPose(startPose);
    }

    public void setDriveVectors() {

    }

    public static PathConstraints getPathConstraints() {
        PathConstraints pc = new PathConstraints(T_VALUE_CONSTRAINT, TIMEOUT_CONSTRAINT, BRAKING_STRENGTH, BRAKING_START);
        PathConstraints.setDefaultConstraints(pc);
        return pc;
    }
}
