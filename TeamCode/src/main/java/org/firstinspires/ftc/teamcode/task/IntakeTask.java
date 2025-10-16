package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class IntakeTask extends TimedTask{

    private final RobotBase robot;
    private final double power;

    private boolean started = false;

    public IntakeTask(RobotBase robot, int finishTimeMillis, double power){
        this.robot = robot;
        this.power = power;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if (!started){
            started = true;
            robot.runIntakeWithPower(power);
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.stopIntake();
    }
}
