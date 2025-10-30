package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class Preset {
    public static SeriesTask createShootTask(RobotBase robot, double FLYWHEEL_VELOCITY, int FLYWHEEL_SHOOT_TIME, int balls){
        SeriesTask task = new SeriesTask(
                new ParallelTask(
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID)
                ),
                new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000)
        );

        for (int i = 1; i <= balls; i++) {
            task.add(
                    new ParallelTask(
                            new UnboundedPusherTask(robot, true),
                            new SeriesTask(
                                    new KickerTask(robot, KickerTask.Position.UP),
                                    new SleepTask(FLYWHEEL_SHOOT_TIME),
                                    //new DelayUntilConditionTask(new ArtifactUnreadyCondition(robot)),
                                    new KickerTask(robot, KickerTask.Position.DOWN)
                            )
                    )

            );

            // not last ball
            if (i != balls) {
                task.add(new SeriesTask(
                        new SleepTask(10),
                        new ParallelTask(
                                new UnboundedPusherTask(robot, false),
                                new UnboundedIntakeTask(robot, 0.4, false),
                                new TimedConditionalTask(new ArtifactReadyCondition(robot), 2000)
                        )
                ));
            }

        }

        return task;
    }

    public static SeriesTask createShootTask(RobotBase robot, double FLYWHEEL_VELOCITY, int FLYWHEEL_SHOOT_TIME){
        return createShootTask(robot, FLYWHEEL_VELOCITY, FLYWHEEL_SHOOT_TIME, 1);
    }
}
