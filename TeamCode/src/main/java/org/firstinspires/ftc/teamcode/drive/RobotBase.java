package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;

public abstract class RobotBase extends MecanumDrive {

    protected final HardwareMap hardwareMap;

    public RobotBase(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(hardwareMap,
                followerConstants,
                driveConstants,
                localizer,
                pathConstraints
        );
        this.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //get hardware
    }

    public void updateEverything() {
        update();
    }
}