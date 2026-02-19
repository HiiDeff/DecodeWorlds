package org.firstinspires.ftc.teamcode.auto;//package org.firstinspires.ftc.teamcode.auto.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.AutoBase;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
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

@Autonomous(name="SplineTest", group="Pedro Test")
@Config
public class SplineTest extends AutoBase {
    public static int AA_NUM_OF_CYCLES = 1;
    public static double MAX_PATH_VELOCITY = 0.9;
    @Override
    protected Location getFirstLocation() {
        return Location.CLOSE;
    }
    @Override
    protected int getNumOfCycles() { return AA_NUM_OF_CYCLES; }
    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        return new SeriesTask();
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;

        SeriesTask task = new SeriesTask();
            task.add(
                new ParallelTask(
                        new RuntimeDrivingTask(robot,
                                builder -> {
                                    Pose pose1 = new Pose(30, 30, Math.PI/2);
                                    Pose pose2 = new Pose(60, 0, Math.PI);
                                    return builder
                                            .addPath(new BezierLine(robot.getPose(), pose1.getPose()))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose1.getHeading())
                                            .addPath(new BezierLine(robot.getPose(), pose2.getPose()))
                                            .setLinearHeadingInterpolation(robot.getHeading(), pose2.getHeading())
                                            .build();
                                }
                        )
                )
        );

        return task;
    }

    @Override
    protected Task createFinishTask() {
        return new SeriesTask();
    }

    @Override
    protected boolean isRed() {
        return false;
    }

    protected boolean isFar() {
        return false;
    }

    protected Pose getStartingPose() {
        return new Pose(0,0*getSign(),0*getSign());
    }


    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }
}
