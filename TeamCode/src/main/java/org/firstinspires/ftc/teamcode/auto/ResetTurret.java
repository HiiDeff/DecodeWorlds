package org.firstinspires.ftc.teamcode.auto;

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

@Autonomous(name = "Reset Robot", group = "Auto AAA_Utils")
public class ResetTurret extends AutoBase {
    @Override
    protected Location getFirstLocation() {
        return Location.CLOSE;
    }
    @Override
    protected int getNumOfCycles() { return 0; }

    @Override
    protected Task createStartTask() {
        state = AutoState.START;
        return new SeriesTask();
    }

    @Override
    protected Task createCycleTask() {
        state = AutoState.CYCLE;
        return new SeriesTask();
    }

    @Override
    protected Task createFinishTask() {
        return new SeriesTask();
    }

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0*getSign(),-Math.PI*getSign());
    }

    @Override
    protected boolean isFar() {
        return false;
    }

    @Override
    protected boolean isRed(){
        return false;
    }

}
