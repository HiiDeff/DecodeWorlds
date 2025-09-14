package org.firstinspires.ftc.teamcode.auto.experimental;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.Task;

public abstract class TestAuto extends AutoBase {

    @Override
    protected Task createStartTask() {
        SeriesTask task = new SeriesTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            return builder.addPath(
                                            new BezierCurve(
                                                    new Pose(72.000, 36.000),
                                                    new Pose(108.000, 36.000),
                                                    new Pose(108.000, 72.000)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-180))
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(108.000, 72.000),
                                                    new Pose(108.000, 108.000),
                                                    new Pose(72.000, 108.000)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(72.000, 108.000),
                                                    new Pose(36.000, 108.000),
                                                    new Pose(36.000, 72.000)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                                    .addPath(
                                            new BezierCurve(
                                                    new Pose(36.000, 72.000),
                                                    new Pose(36.000, 36.000),
                                                    new Pose(72.000, 36.000)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                                    .build();
                        }
                )
        );
        return task;
    }

    @Override
    protected Task createCycleTask() {
        return new SeriesTask();
    }

    @Override
    protected Task createFinishTask() {
        return new SeriesTask();
    }

    protected abstract BezierCurve getBezierCurve1();
    protected abstract BezierCurve getBezierCurve2();
    protected abstract BezierCurve getBezierCurve3();
    protected abstract BezierCurve getBezierCurve4();
}
