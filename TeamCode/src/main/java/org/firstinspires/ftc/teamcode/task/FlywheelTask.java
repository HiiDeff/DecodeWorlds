package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class FlywheelTask extends TimedTask {

    private RobotBase robot;

    private boolean started = false;

    private double flywheelSpeed;

    public FlywheelTask(RobotBase robot, int finishTimeMillis, double flywheelSpeed){
        this.robot = robot;
        this.flywheelSpeed = flywheelSpeed;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if(!started){
            started = true;
            robot.setFlywheelSpeed(flywheelSpeed);
        }
        return false;
    }

    @Override
    public void cancel(){robot.stopFlywheel();}
}
