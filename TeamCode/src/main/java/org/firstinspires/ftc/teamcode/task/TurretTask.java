package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class TurretTask extends TimedTask {
    private RobotBase robot;
    private boolean started = false;
    private double angle;

    public TurretTask(RobotBase robot, double angle, int finishTimeMillis){
        this.robot = robot;
        this.angle = angle;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if(!started){
            started = true;
            robot.setTurretTargetPosition(angle);
            return false;
        }
        return robot.turret.isDone();
    }

    @Override
    public void cancel(){
        // do nothing to maintain velocity
    }
}
