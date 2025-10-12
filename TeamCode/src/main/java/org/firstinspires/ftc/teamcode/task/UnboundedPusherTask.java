package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedPusherTask implements Task, UnboundedTask {
    private boolean started = false;
    private final RobotBase robot;
    private final double position;

    public UnboundedPusherTask(RobotBase robot, double position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean perform(){
        if (!started){
            robot.setPusherPosition(position);
            started = true;
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.setPusherPosition(position);
    }
}
