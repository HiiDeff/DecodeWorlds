package org.firstinspires.ftc.teamcode.task;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public class Presets {
    public static int KICKER_SLEEP = 300;
    public static int SHOOT_TIME = 2000;
    public static double ANGLE = -0.065;

    //assume velocity and pivot position are already set
    public static SeriesTask createShootTask(RobotBase robot, int targetRPM, int ballCnt, PivotTask.Position pivotPos){

        return new SeriesTask(
                new ParallelTask(
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT,pivotPos),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, pivotPos),
                        new FlywheelTask(robot, targetRPM, 2000),
                        new TurretTask(robot, ANGLE, 2000)
                ),
                new ParallelTask(
                        new UnboundedIntakeTask(robot, 1.0, false),
                        new BlockerTask(robot, BlockerTask.Position.OPEN),
                        new RampTask(robot, RampTask.Position.UP),
                        new SleepTask(SHOOT_TIME)
                )
        );

//        SeriesTask task = new SeriesTask();
//        while(--ballCnt>=0) {
//            FarAuto.ballnum += 1;
//            if(ballCnt>0) {
//                task.add(
//                        new ParallelTask(
//                                new FlywheelTask(robot, targetRPM, 3000),
//                                new SeriesTask( //only works in series task
//                                        new TimedConditionalTask(new ArtifactReadyCondition(robot), 3000)
//                                )
//                        )
//                );
//            }
//        }
//
//        SeriesTask finalTask = new SeriesTask(
//                new ParallelTask(
//                        new UnboundedKickerTask(robot, KickerTask.Direction.STANDARD),
//                        new UnboundedIntakeTask(robot, 0.7, false),
//                        task
//
//                ));
//        return finalTask;
    }

    public static SeriesTask createShootTask(RobotBase robot, int targetRPM, PivotTask.Position pivotPos){
        return createShootTask(robot, targetRPM, 1, pivotPos);
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
