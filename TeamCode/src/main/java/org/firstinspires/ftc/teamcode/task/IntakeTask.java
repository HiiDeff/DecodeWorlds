package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class IntakeTask extends TimedTask{

    private final RobotBase robot;
    private final double power;
    private boolean started = false;
    private boolean reversed;

    public IntakeTask(RobotBase robot, double power, boolean reversed, int finishTimeMillis){
        this.robot = robot;
        this.power = power;
        this.reversed = reversed;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
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
