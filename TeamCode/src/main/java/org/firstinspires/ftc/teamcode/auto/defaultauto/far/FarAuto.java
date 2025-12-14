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
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;

@Config
public abstract class FarAuto extends AutoBase {
    public static int FLYWHEEL_VELOCITY = 3850;
    public static double INTAKE_VELOCITY_CONSTRAINT = 0.5;
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
                                    Pose pose = getShootStartPose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(),pose))
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
        task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.FAR));
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
                                    Pose pose = getIntakePose();
                                    return builder
                                            .addPath(getLocation()==Location.LOADING_ZONE? new BezierCurve(robot.getPose(), new Pose(5, -50*getSign()), pose):new BezierCurve(robot.getPose(),pose))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                },
                                (getLocation()==Location.LOADING_ZONE? 0.7:1.0),
                                (getLocation()==Location.LOADING_ZONE? 2000:30000)
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
                                (getLocation()==Location.LOADING_ZONE? 1000:30000)
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
//                                    Pose pose = getShoot2Pose();
//                                    if(cycleNumber==2) pose = getShoot3Pose();
//                                    else if(cycleNumber==3) pose = getShoot4Pose();
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
                                }
                        ),
                        new UnboundedIntakeTask(robot, 0.4, false),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 2000)
                )
        );
        task.add(new SleepTask(200));
        task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.FAR));
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
