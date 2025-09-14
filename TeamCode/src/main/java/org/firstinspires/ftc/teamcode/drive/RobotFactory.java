package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.drive_pp.TestRobot1;

@Config
public class RobotFactory {
    public enum Robot {
        ROBOT_1,
        TEST_ROBOT_1
    }
    public static Robot ACTIVE_ROBOT = Robot.ROBOT_1;

    public static Robot getActiveRobot() {
        return ACTIVE_ROBOT;
    }

    public static Follower createRobot(HardwareMap hardwareMap) {
        switch (ACTIVE_ROBOT) {
            case ROBOT_1:
                return new Robot1(hardwareMap);
            case TEST_ROBOT_1:
            default:
                return new TestRobot1(hardwareMap);
        }
    }
}
