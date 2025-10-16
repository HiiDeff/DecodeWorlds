package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class PusherTask extends TimedTask {
    private boolean started = false;
    private final RobotBase robot;
    private final double position;

    public PusherTask(RobotBase robot, int finishTimeMillis, double position){
        this.robot = robot;
        this.position = position;

        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    public boolean performInternal(){
        if (!started){
            robot.setPusherPosition(position);
            started = true;
        }
        return false;
    }

    @Override
    public void cancel(){

    }
}
