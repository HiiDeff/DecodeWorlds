package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class Presets {
    public static int SHOOT_THREE_TIME = 1000;
    public static double FLYWHEEL_ON_INTAKE_POWER = 0.8, FLYWHEEL_WAIT_INTAKE_POWER = -0.2;
    public static int SHOOT_ONE_TIME = 150, SHOOT_ONE_WAIT_TIME = 200;

    public static Task createRapidShootTask(RobotBase robot){
        return new SeriesTask(
                new ParallelTask(
                        new RampTask(robot, RampTask.Position.UP),
                        new BlockerTask(robot, BlockerTask.Position.OPEN),
                        new UnboundedIntakeTask(robot, 0.8, false),
                        new SleepTask(SHOOT_THREE_TIME)
                ),
                new ParallelTask(
                        new RampTask(robot, RampTask.Position.DOWN),
                        new BlockerTask(robot, BlockerTask.Position.CLOSE)
                )
        );
    }
    public static Task createSlowShootTask(RobotBase robot) {
        return new SeriesTask(
                new ParallelTask(
                        new RampTask(robot, RampTask.Position.UP),
                        new BlockerTask(robot, BlockerTask.Position.OPEN)
                ),
                new IntakeTask(robot, FLYWHEEL_ON_INTAKE_POWER, false, SHOOT_ONE_TIME),
                new IntakeTask(robot, FLYWHEEL_WAIT_INTAKE_POWER, false, SHOOT_ONE_WAIT_TIME),
                new IntakeTask(robot, FLYWHEEL_ON_INTAKE_POWER, false, SHOOT_ONE_TIME),
                new IntakeTask(robot, FLYWHEEL_WAIT_INTAKE_POWER, false, SHOOT_ONE_WAIT_TIME),
                new IntakeTask(robot, FLYWHEEL_ON_INTAKE_POWER, false, SHOOT_ONE_TIME),
                new ParallelTask(
                        new RampTask(robot, RampTask.Position.DOWN),
                        new BlockerTask(robot, BlockerTask.Position.CLOSE)
                )
        );
    }

    public static Task createUnjammingTask(RobotBase robot) {
        Task task = new ParallelTask(
                new UnboundedIntakeTask(robot, 1.0, true),
                new FlywheelTask(robot, 2500, 3000),
                new SeriesTask(
                        new ParallelTask(
                                new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.FAR),
                                new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.FAR),
                                new RampTask(robot, RampTask.Position.UP)
                        ),
                        new SleepTask(500),
                        new ParallelTask(
                                new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.CLOSE),
                                new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.CLOSE),
                                new RampTask(robot, RampTask.Position.DOWN)
                        ),
                        new SleepTask(500),
                        new ParallelTask(
                                new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                                new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID)
                        )
                )
        );
        return task;
    }
}
