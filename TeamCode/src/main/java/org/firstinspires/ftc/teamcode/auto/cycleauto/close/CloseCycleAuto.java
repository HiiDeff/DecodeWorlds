package org.firstinspires.ftc.teamcode.auto.cycleauto.close;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TurretTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;

@Config
public abstract class CloseCycleAuto extends AutoBase {
    public static int AA_NUM_OF_CYCLES = 3;
    public static int FLYWHEEL_VELOCITY = 3200;
    public static double INTAKE_IDLE_POWER = 0.3;
    public static double INTAKE_VELOCITY_CONSTRAINT = 0.5;
    @Override
    protected Location getFirstLocation() {
        return Location.CLOSE;
    }
    @Override
    protected int getNumOfCycles() { return AA_NUM_OF_CYCLES; }
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new TurretTask(robot, 0, 500),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID),
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
        task.add(new SleepTask(100));
        task.add(Presets.createRapidShootTask(robot));

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
                                            .addPath(
                                                    new BezierLine(robot.getPose(), pose.getPose())
                                            )
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                },
                                INTAKE_VELOCITY_CONSTRAINT
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
                            new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false)
                    )
            );
            task.add(new SleepTask(300));
        }
        task.add(new SleepTask(100));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShoot2Pose();
                                    if(cycleNumber==2) pose = getShoot3Pose();
                                    else if(cycleNumber==3) pose = getShoot4Pose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        ),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false)
                )
        );
        task.add(new SleepTask(100));
        task.add(Presets.createRapidShootTask(robot));

        if(cycleNumber == AA_NUM_OF_CYCLES) {
            task.add(new SleepTask(10000));
        }
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
                                },
                                0.4
                        )
                )
        );
        return task;
    }

    protected abstract Pose getShoot1Pose();
    protected abstract Pose getShoot2Pose();
    protected abstract Pose getShoot3Pose();
    protected abstract Pose getShoot4Pose();
    protected abstract Pose getIntake1Pose();
    protected abstract Pose getIntake2Pose();
    protected abstract Pose getIntake3Pose();
    protected abstract Pose getIntake1ForwardPose();
    protected abstract Pose getIntake2ForwardPose();
    protected abstract Pose getIntake3ForwardPose();
    protected abstract Pose getGatePose();
    protected abstract Pose getParkPose();

}
