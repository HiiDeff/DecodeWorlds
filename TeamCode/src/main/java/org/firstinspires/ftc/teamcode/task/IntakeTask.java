package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class IntakeTask extends TimedTask{

    RobotBase robot;
    double intakeSpeed;

    private boolean started = false;

    public IntakeTask(RobotBase robot, int finishTimeMillis, double intakeSpeed){
        this.robot = robot;
        this.intakeSpeed = intakeSpeed;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    protected boolean performInternal(){
        if (!started){
            started = true;
            robot.runIntakeWithPower(intakeSpeed);
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.stopIntake();
    }
}
