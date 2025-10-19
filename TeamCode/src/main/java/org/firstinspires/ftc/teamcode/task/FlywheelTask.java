package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class FlywheelTask extends TimedTask {

    private RobotBase robot;
    private boolean started = false;
    private boolean reversed;
    private double velocity;

    public FlywheelTask(RobotBase robot, double velocityRpm, boolean reversed, int finishTimeMillis){
        this.robot = robot;
        this.velocity = velocityRpm;
        this.reversed = reversed;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if(!started){
            started = true;
            robot.setFlywheelTargetVelocity((reversed ? -1 : 1) * velocity);
            return false;
        }
        return robot.flywheelPID.isDone();
    }

    @Override
    public void cancel(){
        // do nothing to maintain velocity
    }
}
