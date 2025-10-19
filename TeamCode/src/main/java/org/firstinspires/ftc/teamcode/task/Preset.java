package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class Preset {
    public static SeriesTask createShootTask(RobotBase robot, double FLYWHEEL_VELOCITY, int FLYWHEEL_SHOOT_TIME){
        return new SeriesTask(
                new ParallelTask(
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID)
                ),
                new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                new KickerTask(robot, KickerTask.Position.UP),
                new SleepTask(FLYWHEEL_SHOOT_TIME),
                new KickerTask(robot, KickerTask.Position.DOWN)
        );
    }
}
