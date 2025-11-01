package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

@Config
public class Presets {
    public static int KICKER_SLEEP = 300;

    //assume velocity and pivot position are already set
    public static SeriesTask createShootTask(RobotBase robot, int targetRPM, int ballCnt){
        SeriesTask task = new SeriesTask();
        while(--ballCnt>=0) {
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    new SleepTask(100),
                                    new KickerTask(robot, KickerTask.Position.UP),
                                    new SleepTask(KICKER_SLEEP)
                            ),
                            new UnboundedPusherTask(robot, true)
                    )
            );
            if(ballCnt>0) {
                task.add(
                        new ParallelTask(
                                new KickerTask(robot, KickerTask.Position.DOWN),
                                new UnboundedPusherTask(robot, false),
                                new UnboundedIntakeTask(robot, 0.8, false),
                                new FlywheelTask(robot, targetRPM, 3000),
                                new SeriesTask( //only works in series task
                                        new TimedConditionalTask(new ArtifactReadyCondition(robot), 2000)
                                )
                        )
                );
            } else {
                task.add(new KickerTask(robot, KickerTask.Position.DOWN));
            }
        }
        return task;
    }

    public static SeriesTask createShootTask(RobotBase robot, int targetRPM){
        return createShootTask(robot, targetRPM, 1);
    }

    public static Task createUnjammingTask(RobotBase robot) {
        Task task = new ParallelTask(
                new UnboundedIntakeTask(robot, 1.0, true),
                new UnboundedPusherTask(robot, true),
                new FlywheelTask(robot, 2500, 3000),
                new SeriesTask(
                        new ParallelTask(
                                new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.FAR),
                                new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.FAR),
                                new KickerTask(robot, KickerTask.Position.UP)
                        ),
                        new SleepTask(500),
                        new ParallelTask(
                                new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.CLOSE),
                                new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.CLOSE),
                                new KickerTask(robot, KickerTask.Position.DOWN)
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
