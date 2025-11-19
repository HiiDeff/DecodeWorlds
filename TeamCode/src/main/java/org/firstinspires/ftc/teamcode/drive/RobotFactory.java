package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.robot2.Robot2;
import org.firstinspires.ftc.teamcode.drive_pp.TestRobot1;
import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;

@Config
public class RobotFactory {
    public enum Robot {
        ROBOT_2,
        TEST_ROBOT_1
    }
    public static Robot ACTIVE_ROBOT = Robot.ROBOT_2;

    public static MecanumDrive createRobot(HardwareMap hardwareMap) {
        switch (ACTIVE_ROBOT) {
            case ROBOT_2:
                return new Robot2(hardwareMap);
            case TEST_ROBOT_1:
            default:
                return new TestRobot1(hardwareMap);
        }
    }
}
