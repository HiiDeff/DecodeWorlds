package org.firstinspires.ftc.teamcode.auto.experimental;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.Preset;
import org.firstinspires.ftc.teamcode.task.PusherTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.task.UnboundedPusherTask;

public abstract class TestAutoPath extends AutoBase {

    public static int INTAKE_FORWARD_DIST = 25;

    public static int FLYWHEEL_VELOCITY_MID = 2500;

    public static int INTAKE_TIME = 1000;
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getShoot1Pose();
                            return builder
                                    .addPath(
                                            new BezierCurve(
                                                    robot.getPose(),
                                                    pose
                                            )
                                    )
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );
        task.add(Preset.createShootTask(robot, FLYWHEEL_VELOCITY_MID, 75, 3));

        //task.add(new SleepTask(3000));
//        task.add(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose pose = getShoot2Pose();
//                            return builder
//                                    .addPath(
//                                            new BezierCurve(
//                                                    robot.getPose(),
//                                                    pose
//                                            )
//                                    )
//                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                    .build();
//                        }
//                )
//        );
//        task.add(new SleepTask(5000));
//        task.add(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose pose = getShoot3Pose();
//                            return builder
//                                    .addPath(
//                                            new BezierCurve(
//                                                    robot.getPose(),
//                                                    pose
//                                            )
//                                    )
//                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                    .build();
//                        }
//                )
//        );
//        task.add(new SleepTask(5000));
        return task;
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        int cycleNumber = autoStates.getCycleNumber();

        SeriesTask task = new SeriesTask();
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getIntake1Pose();
                            if(cycleNumber==2) pose = getIntake2Pose();
                            else if(cycleNumber==3) pose = getIntake3Pose();
                            return builder
                                    .addPath(
                                        new BezierCurve(
                                                robot.getPose(),
                                                pose
                                        )
                                    )
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getIntake1Pose();
                                    if(cycleNumber==2) pose = getIntake2Pose();
                                    else if(cycleNumber==3) pose = getIntake3Pose();
                                    return builder
                                        .addPath(
                                                new BezierCurve(
                                                        pose,
                                                        new Pose(pose.getX(), pose.getY()+INTAKE_FORWARD_DIST*getSign())
                                                )
                                        )
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                }
                        ),
                        new UnboundedIntakeTask(robot, 0.7, false),
                        new UnboundedPusherTask(robot, false)
                )
        );
        task.add(new ParallelTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getShoot2Pose();
                            if(cycleNumber==2) pose = getShoot3Pose();
                            else if(cycleNumber==3) pose = getShoot4Pose();
                            return builder
                                    .addPath(
                                            new BezierCurve(
                                                    robot.getPose(),
                                                    pose
                                            )
                                    )
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                ),
                new FlywheelTask(robot, FLYWHEEL_VELOCITY_MID, 1000)
                )
        );
        task.add(Preset.createShootTask(robot, FLYWHEEL_VELOCITY_MID, 75, 3));
        return task;
    }

    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = getIntake2Pose();
                            return builder
                                    .addPath(
                                            new BezierCurve(
                                                    robot.getPose(),
                                                    pose
                                            )
                                    )
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
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

}
