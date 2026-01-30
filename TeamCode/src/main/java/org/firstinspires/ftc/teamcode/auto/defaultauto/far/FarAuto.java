package org.firstinspires.ftc.teamcode.auto.defaultauto.far;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.Location;
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
import org.firstinspires.ftc.teamcode.task.TurretTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;

@Config
public abstract class FarAuto extends AutoBase {

    public static int AA_NUM_OF_CYCLES = 3;
    public static int FLYWHEEL_VELOCITY = 4000;
    public static double INTAKE_IDLE_POWER = 0.3;
    public static double INTAKE_VELOCITY_CONSTRAINT = 0.5;
    public static double MAX_PATH_VELOCITY = 0.8;
    @Override
    protected Location getFirstLocation() { return Location.MID; }
    @Override
    protected int getNumOfCycles() { return AA_NUM_OF_CYCLES; }
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new ParallelTask(
                        new TurretTask(robot, 0, 500),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 4000),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.FAR),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.FAR),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShootStartPose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(),pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                MAX_PATH_VELOCITY
                        )
                )
        );
        task.add(new SleepTask(100));
        task.add(Presets.createSlowShootTask(robot));

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
                                    Pose pose = getIntakePose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                MAX_PATH_VELOCITY,
                                (getLocation()==Location.LOADING_ZONE? 4000:30000)
                        ),
                        new FlywheelTask(robot, 0, 500),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        task.add(new SleepTask(100));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = getIntakeForwardPose();
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose.getPose()))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                INTAKE_VELOCITY_CONSTRAINT,
                                (getLocation()==Location.LOADING_ZONE? 3000:30000)
                        ),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        if(firstLocation == Location.MID && autoStates.getCycleNumber()==1){
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
                                    },
                                    MAX_PATH_VELOCITY
                            ),
                            new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false)
                    )
            );
            task.add(new SleepTask(1000));
        }
        task.add(new SleepTask(70));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShootPose();
                                    if(firstLocation == Location.MID && autoStates.getCycleNumber()==1){
                                        return builder
                                                .addPath(new BezierCurve(robot.getPose(), new Pose(-32, 5*getSign()), pose.getPose()))
                                                .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                                .build();
                                    }
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                MAX_PATH_VELOCITY
                        ),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 4000)
                )
        );
        task.add(new SleepTask(100));
        task.add(Presets.createSlowShootTask(robot));

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
                                MAX_PATH_VELOCITY
                        )
                )
        );
        return task;
    }

    protected abstract Pose getIntakeMidPose();
    protected abstract Pose getIntakeFarPose();
    protected abstract Pose getIntakeLoadingZonePose();
    protected abstract Pose getGatePose();
    protected abstract Pose getParkPose();
    protected abstract Pose getIntakeMidForwardPose();
    protected abstract Pose getIntakeFarForwardPose();
    protected abstract Pose getIntakeLoadingZoneForwardPose();
    protected abstract Pose getShootMidPose();
    protected abstract Pose getShootFarPose();
    protected abstract Pose getShootStartPose();
    protected abstract Pose getShootLoadingZonePose();

    protected Pose getIntakePose() {
        if(firstLocation == Location.MID && autoStates.getCycleNumber()==1) {
            return getIntakeMidPose();
        } else if ((firstLocation == Location.MID && autoStates.getCycleNumber() == 2) || (firstLocation == Location.FAR && autoStates.getCycleNumber() == 1)) {
            return getIntakeFarPose();
        } else {
            return getIntakeLoadingZonePose();
        }
    }

    protected Pose getIntakeForwardPose() {
        if(firstLocation == Location.MID && autoStates.getCycleNumber()==1) {
            return getIntakeMidForwardPose();
        } else if ((firstLocation == Location.MID && autoStates.getCycleNumber() == 2) || (firstLocation == Location.FAR && autoStates.getCycleNumber() == 1)) {
            return getIntakeFarForwardPose();
        } else {
            return getIntakeLoadingZoneForwardPose();
        }
    }

    protected Pose getShootPose() {
        if(autoStates.getCycleNumber()==0) {
            return getShootStartPose();
        }else if(firstLocation == Location.MID && autoStates.getCycleNumber()==1) {
            return getShootMidPose();
        } else if ((firstLocation == Location.MID && autoStates.getCycleNumber() == 2) || (firstLocation == Location.FAR && autoStates.getCycleNumber() == 1)) {
            return getShootFarPose();
        } else {
            return getShootLoadingZonePose();
        }
    }

    protected Location getLocation() {
        if(firstLocation == Location.MID && autoStates.getCycleNumber()==1) {
            return Location.MID;
        } else if ((firstLocation == Location.MID && autoStates.getCycleNumber() == 2) || (firstLocation == Location.FAR && autoStates.getCycleNumber() == 1)) {
            return Location.FAR;
        } else {
            return Location.LOADING_ZONE;
        }
    }
}
