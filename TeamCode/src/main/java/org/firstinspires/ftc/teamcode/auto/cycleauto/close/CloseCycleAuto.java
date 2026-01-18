package org.firstinspires.ftc.teamcode.auto.cycleauto.close;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
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
    public static int AA_NUM_OF_CYCLES = 4;
    public static boolean INTAKE_FAR_SPIKE_MARK = false;
    public static int INTAKE_AT_GATE_TIME = 800;
    public static int SHOOT_TIME = 600;
    public static int FLYWHEEL_VELOCITY = 3200;
    public static double INTAKE_IDLE_POWER = 0.3;
    public static double INTAKE_VELOCITY_CONSTRAINT = 1.0;
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
                        new TurretTask(robot, -Math.PI/2*getSign(), 2000),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 1000),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.MID),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.MID),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShootPose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                }
                        )
                )
        );
        task.add(Presets.createRapidShootTask(robot, SHOOT_TIME));

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
                                    Pose pose = getIntakeGatePose();
                                    if(cycleNumber==1) pose = getIntake1Pose();
                                    else if(cycleNumber==AA_NUM_OF_CYCLES-1 && INTAKE_FAR_SPIKE_MARK) pose = getIntake2Pose();
                                    else if(cycleNumber==AA_NUM_OF_CYCLES) pose = getIntake3Pose();
                                    return builder
                                            .addPath(cycleNumber<AA_NUM_OF_CYCLES-(INTAKE_FAR_SPIKE_MARK?1:0)? new BezierCurve(robot.getPose(),new Pose(40, 30*getSign()), pose):new BezierCurve(robot.getPose(),pose))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                            .build();
                                },
                                1.0,
                                (cycleNumber==1||cycleNumber==AA_NUM_OF_CYCLES||(INTAKE_FAR_SPIKE_MARK&&cycleNumber==AA_NUM_OF_CYCLES-1)?30000:2000)
                        ),
                        new FlywheelTask(robot, 0, 300)
                )
        );
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = getIntakeGateForwardPose();
                                    if(cycleNumber==1) pose = getIntake1ForwardPose();
                                    else if(cycleNumber==AA_NUM_OF_CYCLES-1 && INTAKE_FAR_SPIKE_MARK) pose = getIntake2ForwardPose();
                                    else if(cycleNumber==AA_NUM_OF_CYCLES) pose = getIntake3ForwardPose();
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose.getPose()))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                },
                                INTAKE_VELOCITY_CONSTRAINT
                        ),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        if(cycleNumber!=1 && cycleNumber!=AA_NUM_OF_CYCLES) {
            task.add(new IntakeTask(robot, 1.0, false, INTAKE_AT_GATE_TIME));
        }
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShootPose();
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

        task.add(Presets.createRapidShootTask(robot, SHOOT_TIME));

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
                        new TurretTask(robot, 0, 300),
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

    protected abstract Pose getShootPose();
    protected abstract Pose getIntake1Pose();
    protected abstract Pose getIntake1ForwardPose();
    protected abstract Pose getIntakeGatePose();
    protected abstract Pose getIntakeGateForwardPose();
    protected abstract Pose getIntake2Pose();
    protected abstract Pose getIntake2ForwardPose();
    protected abstract Pose getIntake3Pose();
    protected abstract Pose getIntake3ForwardPose();
    protected abstract Pose getParkPose();

}
