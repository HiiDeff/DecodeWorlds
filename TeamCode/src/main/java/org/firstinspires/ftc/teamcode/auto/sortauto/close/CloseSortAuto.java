package org.firstinspires.ftc.teamcode.auto.sortauto.close;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.TurretTask;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.util.limelight.AprilTagType;

@Config
public abstract class CloseSortAuto extends AutoBase {

    public static int AA_NUM_OF_CYCLES = 3;
    public static int FLYWHEEL_VELOCITY = 2600;
    public static double INTAKE_IDLE_POWER = 0.3, TURRET_OFFSET = 0.35;
    public static double INTAKE_VELOCITY_CONSTRAINT = 0.8;
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
                        new SeriesTask(
                                new TurretTask(robot, -Math.PI/3*getSign(), 750),
                                new SleepTask(750),
                                new TurretTask(robot, -(Math.PI/4+0.1)*getSign(), 500)
                        ),
                        new FlywheelTask(robot, 3000, 1000),
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
        AprilTagType intakePattern = AprilTagType.MOTIF_PPG;
        if(cycleNumber==2) intakePattern = AprilTagType.MOTIF_PGP;
        else if(cycleNumber==3) intakePattern = AprilTagType.MOTIF_GPP;
        int sortRequirement = robot.limelightAprilTagDetector.getSortRequirement(intakePattern);

        SeriesTask task = new SeriesTask();

        // Go to intake position
        task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose = getIntake1ForwardPose();
                                    if(cycleNumber==2) pose = getIntake2Pose();
                                    else if(cycleNumber==3) pose = getIntake3Pose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(),pose))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                },
                                1.0
                        ),
                        new FlywheelTask(robot, 0, 300),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.CLOSE),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.CLOSE),
                        new UnboundedIntakeTask(robot, 1.0, false)
                )
        );
        if(cycleNumber==2||cycleNumber==3) {
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(robot,
                                    builder -> {
                                        Pose pose = getIntake2ForwardPose();
                                        if(cycleNumber==3) pose = getIntake3ForwardPose();
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
        } else {
            task.add(
                    new ParallelTask(
                            new RuntimeDrivingTask(
                                    robot,
                                    builder -> {
                                        Pose pose = getGate1Pose();
                                        return builder
                                                .addPath(new BezierCurve(robot.getPose(), new Pose(38, 13*getSign()), pose))
                                                .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                                .build();
                                    },
                                    1.0,
                                    3000
                            ),
                            new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false)
                    )
            );
        }
        // Go back to shoot position
        task.add(
                new ParallelTask(
                        new SeriesTask(
                                new RuntimeDrivingTask(
                                        robot,
                                        builder -> {
                                            Pose pose = getShoot2Pose();
                                            if(cycleNumber==2) pose = getShoot3Pose();
                                            else if(cycleNumber==3) pose = getShoot4Pose();
                                            return builder
                                                    .addPath(new BezierCurve(robot.getPose(), pose))
                                                    .setConstantHeadingInterpolation(pose.getHeading())
                                                    .build();
                                        }
                                )
                        ),
                        new FlywheelTask(robot, (sortRequirement==1? 0:FLYWHEEL_VELOCITY), 600),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new TurretTask(robot, (sortRequirement==1? 0:Math.PI/4*getSign()), 600),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.CLOSE),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.CLOSE)
                )
        );

        if(sortRequirement==1) {
            task.add(
                    new RuntimeDrivingTask(
                            robot,
                            builder -> {
                                Pose pose = getShoot2SpinPose();
                                if(cycleNumber==2) pose = getShoot3SpinPose();
                                else if(cycleNumber==3) pose = getShoot4SpinPose();
                                return builder
                                        .addPath(new BezierCurve(robot.getPose(), pose))
                                        .setConstantHeadingInterpolation(pose.getHeading())
                                        .build();
                            }
                    )
            );
            task.add(
                    new ParallelTask(
                            new SeriesTask(
                                    Presets.createSortOneTask(robot),
                                    new IntakeTask(robot, 1.0, false, 1000)
                            ),
                            new SeriesTask(
                                    new SleepTask(1500),
                                    new RuntimeDrivingTask(
                                            robot,
                                            builder -> {
                                                Pose pose = getShoot2SpinPose();
                                                if(cycleNumber==2) pose = getShoot3SpinPose();
                                                else if(cycleNumber==3) pose = getShoot4SpinPose();
                                                pose = new Pose(pose.getX()+3.5, pose.getY()+3.5*getSign(), pose.getHeading());
                                                return builder
                                                        .addPath(new BezierCurve(robot.getPose(), pose))
                                                        .setConstantHeadingInterpolation(pose.getHeading())
                                                        .build();
                                            },
                                            0.2,
                                            1000
                                    )
                            )
                    )
            );
        }

        task.add(
                new ParallelTask(
                        new TurretTask(robot, (cycleNumber==3?Math.PI/4:(Math.PI/4-TURRET_OFFSET))*getSign(), 600),
                        new RuntimeDrivingTask(
                                robot,
                                builder -> {
                                    Pose pose = getShoot2GuaranteePose();
                                    if(cycleNumber==2) pose = getShoot3GuaranteePose();
                                    else if(cycleNumber==3) pose = getShoot4GuaranteePose();
                                    return builder
                                            .addPath(new BezierCurve(robot.getPose(), pose))
                                            .setConstantHeadingInterpolation(pose.getHeading())
                                            .build();
                                }
                        ),
                        new FlywheelTask(robot, FLYWHEEL_VELOCITY, 600),
                        new UnboundedIntakeTask(robot, INTAKE_IDLE_POWER, false),
                        new PivotTask(robot, PivotTask.WhichPivot.LEFT, PivotTask.Position.CLOSE),
                        new PivotTask(robot, PivotTask.WhichPivot.RIGHT, PivotTask.Position.CLOSE)
                )
        );

        task.add(Presets.createRapidShootTask(robot, 1200, 0.4));

//        if(cycleNumber == AA_NUM_OF_CYCLES) {
//            task.add(new SleepTask(10000));
//        }
        return task;
    }

    @Override
    protected Task createFinishTask() {
        state = AutoState.FINISH;
        SeriesTask task = new SeriesTask();
        task.add(
                new FlywheelTask(robot, 0,1000)
        );
//        task.add(new SleepTask(10000));
        return task;
    }

    protected abstract Pose getIntake2Pose();
    protected abstract Pose getIntake3Pose();
    protected abstract Pose getIntake1ForwardPose();
    protected abstract Pose getIntake2ForwardPose();
    protected abstract Pose getIntake3ForwardPose();
    protected abstract Pose getGate1Pose();
    protected abstract Pose getShoot1Pose();
    protected abstract Pose getShoot2Pose();
    protected abstract Pose getShoot3Pose();
    protected abstract Pose getShoot4Pose();
    protected abstract Pose getShoot2SpinPose();
    protected abstract Pose getShoot3SpinPose();
    protected abstract Pose getShoot4SpinPose();

    protected abstract Pose getShoot2GuaranteePose();
    protected abstract Pose getShoot3GuaranteePose();
    protected abstract Pose getShoot4GuaranteePose();
    protected abstract Pose getParkPose();

}
