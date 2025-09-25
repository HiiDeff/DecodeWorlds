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
    private final Pose startPose = new Pose(0,0,0.0);

    public MecanumDrive(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(followerConstants,
                localizer,
                new Mecanum(hardwareMap, driveConstants),
                pathConstraints
        );
    }

    public void init() {
        setStartingPose(startPose);
    }

    public void setDriveVectors() {

    }
}
