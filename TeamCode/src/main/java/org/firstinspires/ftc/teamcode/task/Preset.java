package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class Preset {
    public static SeriesTask createShootTask(RobotBase robot, double FLYWHEEL_VELOCITY, int FLYWHEEL_SHOOT_TIME){
        return new SeriesTask(
                new FlywheelTask(robot, FLYWHEEL_VELOCITY, false, 1000),
                new KickerTask(robot, KickerTask.Position.UP),
                new SleepTask(FLYWHEEL_SHOOT_TIME),
                new KickerTask(robot, KickerTask.Position.DOWN)
        );
    }
}
