package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class UnboundedIntakeTask implements UnboundedTask, Task{

    private final RobotBase robot;
    private final double power;
    private boolean started = false;
    private boolean reversed;

    public UnboundedIntakeTask(RobotBase robot, double power, boolean reversed){
        this.robot = robot;
        this.power = power;
        this.reversed = reversed;
    }

    @Override
    public boolean perform(){
        if (!started){
            started = true;
            robot.runIntakeWithPower((reversed ? -1 : 1) * power);
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.stopIntake();
    }
}
