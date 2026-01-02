package org.firstinspires.ftc.teamcode.auto.experimental;//package org.firstinspires.ftc.teamcode.auto.experimental;

import android.util.Log;

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
public abstract class TestAutoPath extends AutoBase {
    public static int FLYWHEEL_VELOCITY = 2000;

    public static boolean takeFarBalls = true;

    @Override
    protected Location getFirstLocation() {
        return Location.MID;
    }
    @Override
    protected Task createStartTask() {
//        state = AutoState.START;
//        SeriesTask task = new SeriesTask();
//        task.add(
//            new ParallelTask(
//                    new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
//                    new RuntimeDrivingTask(
//                            robot,
//                            builder -> {
//                                Pose pose = getShoot1Pose();
//                                return builder
//                                        .addPath(new BezierCurve(robot.getPose(),pose))
//                                        .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                        .build();
//                            }
//                    )
//            )
//        );
//        //task.add(new SleepTask(5000));
//        /*task.add(
//                new RuntimeDrivingTask(
//                        robot,
//                        builder -> {
//                            Pose pose = getShoot2Pose();
//                            return builder
//                                    .addPath(new BezierCurve(robot.getPose(),pose))
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
//                                    .addPath(new BezierCurve(robot.getPose(),pose))
//                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                    .build();
//                        }
//                )
//        );
//        task.add(new SleepTask(500000));
//        task.add(
//                new ParallelTask(
//                        new RuntimeDrivingTask(
//                                robot,
//                                builder -> {
//                                    Pose pose = getShoot1Pose();
//                                    return builder
//                                            .addPath(new BezierCurve(robot.getPose(),pose))
//                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                            .build();
//                                }
//                        ),
//                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 3000),
//                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
//                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID)
//                )
//        );*/
//        task.add(new SleepTask(100));
//        task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.FAR));
//        task.add(new SeriesTask(
//                new BlockerTask(robot, BlockerTask.Position.CLOSE),
//                new RampTask(robot, RampTask.Position.DOWN)
//        ));
//
//        if(takeFarBalls){
//            task.add(
//                    new ParallelTask(
//                            new RuntimeDrivingTask(robot, builder -> {
//                                Pose pose = getIntake1Pose();
//                                return builder.addPath(new BezierCurve(robot.getPose(), pose))
//                                        .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                        .build();
//                            }),
//                            new FlywheelTask(robot, 0, 500)
//                    )
//            );
//            task.add(
//                    new ParallelTask(
//                            new RuntimeDrivingTask(robot, builder -> {
//                                Pose pose = getIntake1ForwardPose();
//                                return builder.addPath(new BezierLine(robot.getPose(), pose))
//                                        .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                        .build();
//                            }),
//                            new UnboundedIntakeTask(robot, 1.0, false)
//                    )
//            );
//            task.add(new SleepTask(200));
//            task.add(
//                    new ParallelTask(
//                            new RuntimeDrivingTask(robot, builder -> {
//                                Pose pose = getShoot2Pose();
//                                return builder.addPath(new BezierCurve(robot.getPose(), pose))
//                                        .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                        .build();
//                            }),
//                            new FlywheelTask(robot, FLYWHEEL_VELOCITY, 2500)
//                    )
//            );
//            task.add(new SleepTask(200));
//            task.add(Presets.createShootTask(robot, FLYWHEEL_VELOCITY, 3, PivotTask.Position.FAR));
//            task.add(new SeriesTask(
//                    new BlockerTask(robot, BlockerTask.Position.CLOSE),
//                    new RampTask(robot, RampTask.Position.DOWN)
//            ));
//            autoStates.setCycleNumber(2);
//        }
//
//        return task;
        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);
        robot.startArtifactPipeline();
        robot.updateLimelight();
        return new SeriesTask();
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        int cycleNumber = autoStates.getCycleNumber();
        SeriesTask task = new SeriesTask();
        robot.updateLimelight();
//        task.add(
//                new ParallelTask(
//                        new RuntimeDrivingTask(robot,
//                                builder -> {
//                                    Pose pose = robot.getLargestArtifactClusterPose();
//                                    return builder
//                                            .addPath(new BezierCurve(robot.getPose(),pose))
//                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                            .build();
//                                }
//                        ),
//                        new FlywheelTask(robot, 0, 500)
//                )
//        );
        task.add(new SleepTask(100));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = robot.getTargetArtifactClusterPose();
                                    Log.e("adbug test auto pose", pose.getX()+" "+pose.getY()+" "+pose.getHeading());
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose.getPose()))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                },
                                1.0,
                                10000
                        ),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        /*if(cycleNumber==1){
        task.add(
                new SeriesTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getGatePose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );}*/
        task.add(new SleepTask(200));
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = new Pose(0,0, Math.PI);
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        ),
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
//        state = AutoState.FINISH;
//        SeriesTask task = new SeriesTask();
//        task.add(
//                new ParallelTask(
//                        new FlywheelTask(robot, 0,1000),
//                        new RuntimeDrivingTask(
//                                robot,
//                                builder -> {
//                                    Pose pose = getParkPose();
//                                    return builder
//                                            .addPath(new BezierCurve(robot.getPose(), pose))
//                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
//                                            .build();
//                                }
//                        )
//                )
//        );
        return new SeriesTask();
    }

    protected abstract Pose getShoot1Pose();
    protected abstract Pose getIntake1Pose();
    protected abstract Pose getShoot2Pose();
    protected abstract Pose getIntake2Pose();
    protected abstract Pose getShoot3Pose();
    protected abstract Pose getIntake3Pose();
    protected abstract Pose getShoot4Pose();
    protected abstract Pose getIntake4Pose();
    protected abstract Pose getGatePose();
    protected abstract Pose getParkPose();
    protected abstract Pose getIntake1ForwardPose();
    protected abstract Pose getIntake2ForwardPose();

    protected abstract Pose getIntake3ForwardPose();
    protected abstract Pose getIntake4ForwardPose();
}
