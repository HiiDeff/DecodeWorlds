package org.firstinspires.ftc.teamcode.drive_pp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;

@Config
public class TestRobot1 extends MecanumDrive {

    public static FollowerConstants FOLLOWER_CONSTANTS
            = new FollowerConstants()
            .mass(10.65);

    public static MecanumConstants DRIVE_CONSTANTS
            = new MecanumConstants()
            .maxPower(1.0);

    public static PinpointConstants PINPOINT_CONSTANTS
            = new PinpointConstants()
            .strafePodX(0.5);

    public TestRobot1(HardwareMap hardwareMap) {
        super(hardwareMap,
                FOLLOWER_CONSTANTS,
                DRIVE_CONSTANTS,
                new PinpointLocalizer(hardwareMap, PINPOINT_CONSTANTS)
        );
    }
}
