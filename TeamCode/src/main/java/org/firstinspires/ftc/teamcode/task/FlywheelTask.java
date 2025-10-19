package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class FlywheelTask extends TimedTask {

    private RobotBase robot;
    private boolean started = false;
    private double velocityRpm;

    public FlywheelTask(RobotBase robot, double velocityRpm, int finishTimeMillis){
        this.robot = robot;
        this.velocityRpm = velocityRpm;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if(!started){
            started = true;
            robot.setFlywheelTargetVelocity(velocityRpm);
            return false;
        }
        return robot.flywheelPID.isDone();
    }

    @Override
    public void cancel(){
        // do nothing to maintain velocity
    }
}
