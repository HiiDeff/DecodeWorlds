package org.firstinspires.ftc.teamcode.auto.defaultauto.close;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;

@Config
public abstract class CloseAuto extends AutoBase {
    public static int FLYWHEEL_VELOCITY = 2900;
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShoot1Pose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );
        //task.add(new SleepTask(5000));
        /*task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getShoot2Pose();
                            return builder
                                    .addPath(new BezierCurve(robot.getPose(),pose))
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );
        task.add(new SleepTask(5000));
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getShoot3Pose();
                            return builder
                                    .addPath(new BezierCurve(robot.getPose(),pose))
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );
        task.add(new SleepTask(500000));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShoot1Pose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(),pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        ),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 3000),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID)
                )
        );*/
        task.add(new SleepTask(100));
        task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.MID));
        task.add(new SeriesTask(
                new BlockerTask(robot, BlockerTask.Position.CLOSE),
                new RampTask(robot, RampTask.Position.DOWN)
        ));
        return task;
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        int cycleNumber = autoStates.getCycleNumber();
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = getIntake1Pose();
                                    if(cycleNumber==2) pose = getIntake2Pose();
                                    else if(cycleNumber==3) pose = getIntake3Pose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(),pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        ),
                        new FlywheelTask(robot, 0, 300)
                )
        );
        task.add(new SleepTask(100));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = getIntake1ForwardPose();
                                    if(cycleNumber==2) pose = getIntake2ForwardPose();
                                    else if(cycleNumber==3) pose = getIntake3ForwardPose();
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose.getPose()))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                }
                        ),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        if(cycleNumber==1){
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(
                                    robot,
                                    builder -> {
                                        Pose pose = getGatePose();
                                        return builder
                                                .addPath(new BezierCurve(robot.getPose(), pose))
                                                .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                                .build();
                                    }
                            ),
                            new UnboundedIntakeTask(robot, 0.4, false)
                    )
            );
            task.add(new SleepTask(300));
        }
        task.add(new SleepTask(70));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShoot2Pose();
                                    if(cycleNumber==2) pose = getShoot3Pose();
                                    else if(cycleNumber==3) pose = getShoot4Pose();
                                    if(cycleNumber==1){
                                        return builder
                                                .addPath(new BezierCurve(robot.getPose(), new Pose(-80, -20*getSign()), pose.getPose()))
                                                .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                                .build();
                                    }
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        ),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new UnboundedIntakeTask(robot, 0.4, false)
                )
        );
        task.add(new SleepTask(100));
        task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.MID));
        task.add(new SeriesTask(
                new BlockerTask(robot, BlockerTask.Position.CLOSE),
                new RampTask(robot, RampTask.Position.DOWN)
        ));
        return task;
    }

    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new FlywheelTask(robot, 0,1000),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getParkPose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );
        return task;
    }

    protected abstract Pose getShoot1Pose();
    protected abstract Pose getIntake1Pose();
    protected abstract Pose getShoot2Pose();
    protected abstract Pose getIntake2Pose();
    protected abstract Pose getShoot3Pose();
    protected abstract Pose getIntake3Pose();
    protected abstract Pose getShoot4Pose();
    protected abstract Pose getGatePose();
    protected abstract Pose getParkPose();
    protected abstract Pose getIntake1ForwardPose();
    protected abstract Pose getIntake2ForwardPose();
    protected abstract Pose getIntake3ForwardPose();

}
