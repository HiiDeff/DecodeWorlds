package org.firstinspires.ftc.teamcode.task;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.PathActioner;

public class RuntimeDrivingTask implements Task {

    private final Follower robot;
    private final PathActioner actioner;

    private double maxPower = 1.0;
    private boolean started = false;
    private PathChain path;

    public RuntimeDrivingTask(Follower robot, PathActioner actioner){
        this.robot = robot;
        this.actioner = actioner;
    }

    public RuntimeDrivingTask(Follower robot, PathActioner actioner, double maxPower) {
        this.robot = robot;
        this.actioner = actioner;
        this.maxPower  = maxPower;
    }

    @Override
    public boolean perform() {
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
}