package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

@Config
public class Presets {
    public static int KICKER_SLEEP = 500, DELAY_BETWEEN_SHOTS = 500;

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
}
