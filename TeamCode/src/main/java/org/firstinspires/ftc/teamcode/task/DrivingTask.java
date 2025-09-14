package org.firstinspires.ftc.teamcode.task;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.PathActioner;

public class DrivingTask implements Task {

    private final Follower robot;
    private final PathChain path;
    private boolean started = false;

    public DrivingTask(Follower robot, PathActioner actioner) {
        this.robot = robot;
        this.path = actioner.createPath(robot.pathBuilder());
    }

    @Override
    public boolean perform() {
        if(!started) {
            started = true;
            robot.followPath(path);
        }
        return !robot.isBusy();
    }
}
