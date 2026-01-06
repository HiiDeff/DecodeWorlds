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
import org.firstinspires.ftc.teamcode.util.limelight.Coords;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public abstract class TestAutoPath extends AutoBase {
    public static int FLYWHEEL_VELOCITY = 4000;

    public static double INTAKE_OFFSET_X = 0, INTAKE_OFFSET_Y = -15;
    public static double INTAKE_FORWARD_OFFSET_X = 0, INTAKE_FORWARD_OFFSET_Y = 0;
    public static double INTAKE_BACK_OFFSET_X = 0, INTAKE_BACK_OFFSET_Y = -20;

    public static boolean takeFarBalls = true;

    @Override
    protected Location getFirstLocation() {
        return Location.MID;
    }
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        SeriesTask task = new SeriesTask();
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

        task.add(
                new ParallelTask(
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new UnboundedIntakeTask(robot, 0.3, false),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = new Pose(-8.493, -0.686, -Math.PI/2);
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );

        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);
        robot.startArtifactPipeline();
        robot.updateLimelight();
        return new SeriesTask();
    }

    Pose getNewPose(Pose start, Pose end, Pose offset){
        double heading = Math.atan2(end.getY() - start.getY(), end.getX() - start.getX());

        double x = end.getX() + offset.getY() * Math.cos(heading) - offset.getX() * Math.sin(heading);
        double y = end.getY() + (offset.getY() * Math.sin(heading) + offset.getX() * Math.cos(heading));

        return new Pose(x, y);
    }

    Pose getNewPose(Pose start, Pose end, Pose offset, double heading){
        double x = end.getX() + offset.getY() * Math.cos(heading) - offset.getX() * Math.sin(heading);
        double y = end.getY() + (offset.getY() * Math.sin(heading) + offset.getX() * Math.cos(heading));

        return new Pose(x, y);
    }

    BezierLine getBezierLine(Pose start, Pose end, Pose offset) {
        return new BezierLine(start, getNewPose(start, end, offset));
    }

    BezierLine getBezierLine(Pose start, Pose end, Pose offset, double heading) {
        return new BezierLine(start, getNewPose(start, end, offset, heading));
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

        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = new Pose(-8.493,  -0.686, -Math.PI);
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );

        task.add(new SleepTask(100));

        List<Pose> poses = robot.getTopThreeTargetPositions();

        for (int i = 0; i < 3; i++){
            Pose currentPose = poses.get(i);

            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(robot,
                                    builder -> {
                                        Log.e("adbug test auto pose", currentPose.toString());
                                        return builder
                                                .addPath(getBezierLine(robot.getPose(), currentPose.getPose(), new Pose(INTAKE_OFFSET_X, INTAKE_OFFSET_Y)))
                                                .setTangentHeadingInterpolation()
                                                .build();
                                    },
                                    1.0,
                                    3000
                            ),
                            new UnboundedIntakeTask(robot, 1.0, false)
                    )
            );
            task.add(new SleepTask(100));
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(robot,
                                    builder -> {
                                        Log.e("adbug test auto moving forward", currentPose.toString());
                                        return builder
                                                .addPath(getBezierLine(robot.getPose(), currentPose.getPose(), new Pose(INTAKE_FORWARD_OFFSET_X, INTAKE_FORWARD_OFFSET_Y)))
                                                .setTangentHeadingInterpolation()
                                                .build();
                                    },
                                    0.7,
                                    2000
                            ),
                            new UnboundedIntakeTask(robot, 1.0, false)
                    )
            );
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(robot,
                                    builder -> {
                                        Log.e("adbug test auto moving back", currentPose.toString());
                                        return builder
                                                .addPath(getBezierLine(robot.getPose(), currentPose.getPose(), new Pose(INTAKE_BACK_OFFSET_X, INTAKE_BACK_OFFSET_Y), robot.getHeading()))
                                                .setLinearHeadingInterpolation(robot.getHeading(), robot.getHeading())
                                                .build();
                                    },
                                    1.0,
                                    2000
                            ),
                            new UnboundedIntakeTask(robot, 1.0, false)
                    )
            );
            task.add(new SleepTask(100));
        }

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
        Log.e("adbug test auto", "Moving back");
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = new Pose(-8.493,  -0.686, -2.75);
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


}

class Compare implements Comparator<Pose>{
    @Override
    public int compare(Pose a, Pose b){
        return (int)(a.getX() - b.getX());
    }
}
