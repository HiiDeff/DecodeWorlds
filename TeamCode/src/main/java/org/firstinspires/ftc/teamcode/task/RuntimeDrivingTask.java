package org.firstinspires.ftc.teamcode.task;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.PathActioner;

public class RuntimeDrivingTask extends TimedTask {

    private final Follower robot;
    private final PathActioner actioner;
    private double maxPower = 1.0;
    private boolean started = false;
    private PathChain path;

    public RuntimeDrivingTask(Follower robot, PathActioner actioner){
        this.robot = robot;
        this.actioner = actioner;
        setFinishTimeMillis(8000);
    }
    public RuntimeDrivingTask(Follower robot, PathActioner actioner, double maxPower) {
        this.robot = robot;
        this.actioner = actioner;
        this.maxPower  = maxPower;
        setFinishTimeMillis(8000);
    }
    public RuntimeDrivingTask(Follower robot, PathActioner actioner, double maxPower, int timeoutMs){
        this.robot = robot;
        this.actioner = actioner;
        this.maxPower = maxPower;
        setFinishTimeMillis(timeoutMs);
    }

    @Override
    protected boolean performInternal() {
        if(!started) {
            started = true;
            path = actioner.createPath(robot.pathBuilder());
            Log.i("el_debug started", ""+(path==null));
            robot.setMaxPower(maxPower);
            robot.followPath(path);
        }
        Log.i("el_debug performing",""+robot.isBusy());
        return !robot.isBusy();
    }

    @Override
    public void cancel() {
    }
}