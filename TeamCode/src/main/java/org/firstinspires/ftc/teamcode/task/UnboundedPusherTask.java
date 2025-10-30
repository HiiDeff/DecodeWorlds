package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedPusherTask implements UnboundedTask, Task{
    private boolean started = false;
    private final RobotBase robot;
    private final boolean reversed;

    public UnboundedPusherTask(RobotBase robot, boolean reversed){
        this.robot = robot;
        this.reversed = reversed;
    }

    @Override
    public boolean perform(){
        if (!started){
            if(!reversed) robot.runPusher();
            else robot.runPusherReversed();
            started = true;
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.stopPusher();
    }
}
