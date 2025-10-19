package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class PusherTask extends TimedTask {
    private boolean started = false;
    private final RobotBase robot;
    private final boolean reversed;

    public PusherTask(RobotBase robot, boolean reversed, int finishTimeMillis){
        this.robot = robot;
        this.reversed = reversed;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    public boolean performInternal(){
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
