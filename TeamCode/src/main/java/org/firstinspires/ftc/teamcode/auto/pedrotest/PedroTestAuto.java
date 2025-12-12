package org.firstinspires.ftc.teamcode.auto.pedrotest;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.task.RuntimeDrivingTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
@Autonomous(name = "PedroTestAuto", group = "autoTest")
public class PedroTestAuto extends AutoBase {

    public static double
        POS1_X, POS1_Y, POS1_H,
        POS2_X, POS2_Y, POS2_H,
        POS3_X, POS3_Y, POS3_H;

    static {
        POS1_X = 48.5; POS1_Y = 52.4;  POS1_H = Math.PI/2;
        POS2_X = 65.8; POS2_Y = -22.7; POS2_H = -Math.PI/2;
        POS3_X = 2.1;  POS3_Y = 1.1;   POS3_H = -Math.PI/2;
    }
    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Task createStartTask() {
        SeriesTask task = new SeriesTask(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = new Pose(POS1_X, POS1_Y, POS1_H);
                            return builder
                                    .addPath(new BezierCurve(robot.getPose(), pose))
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );

        task.add(new SleepTask(10000));

        task.add(new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = new Pose(POS2_X, POS2_Y, POS2_H);
                            return builder
                                    .addPath(new BezierCurve(robot.getPose(), pose))
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );

        task.add(new SleepTask(10000));

        task.add(
                new RuntimeDrivingTask(
                        robot,
                        builder -> {
                            Pose pose = new Pose(POS3_X, POS3_Y, POS3_H);
                            return builder
                                    .addPath(new BezierCurve(robot.getPose(), pose))
                                    .setLinearHeadingInterpolation(robot.getHeading(), pose.getHeading())
                                    .build();
                        }
                )
        );

        task.add(new SleepTask(10000));

        return task;
    }

    @Override
    protected Task createCycleTask() {
        return null;
    }

    @Override
    protected Task createFinishTask() {
        return null;
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.toRadians(0));
    }

    @Override
    protected boolean isRed() {
        return false;
    }

    @Override
    protected boolean isFar() {
        return false;
    }
}
