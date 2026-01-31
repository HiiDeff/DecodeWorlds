package org.firstinspires.ftc.teamcode.auto.visionauto.far;

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
public abstract class FarVisionAuto extends AutoBase {

    public static int AA_NUM_OF_CYCLES = 3;
    public static int FLYWHEEL_VELOCITY = 3950;
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
        robot.startArtifactPipeline();
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
                                    Pose pose = getShootingPose();
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

        // Vision Part: See where the balls are
        Location target = robot.getArtifactDensestLocation();

        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = (target == Location.LOADING_ZONE) ? getLoadingZoneIntakePose() : getGateIntakePose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                MAX_PATH_VELOCITY,
                                4000
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
                                    Pose pose = (target == Location.LOADING_ZONE) ? getLoadingZoneIntakeForwardPose() : getGateIntakeForwardPose();
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose.getPose()))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                INTAKE_VELOCITY_CONSTRAINT,
                                3000
                        ),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShootingPose();
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

    protected abstract Pose getParkPose();
    protected abstract Pose getShootingPose();
    protected abstract Pose getLoadingZoneIntakePose();
    protected abstract Pose getGateIntakePose();
    protected abstract Pose getLoadingZoneIntakeForwardPose();
    protected abstract Pose getGateIntakeForwardPose();


}
