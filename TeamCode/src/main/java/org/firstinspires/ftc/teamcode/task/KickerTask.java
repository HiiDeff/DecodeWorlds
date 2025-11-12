package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class KickerTask extends TimedTask {
    private final RobotBase robot;
    private final Direction direction;
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private boolean started = false;

    public KickerTask(RobotBase robot, Direction direction, int finishTimeMillis){
        this.robot = robot;
        this.direction = direction;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    public boolean performInternal(){
        if (!started){
            robot.kicker.setPower(robot.getKickerPower(direction));
            started = true;
        }
        return false;
    }

    public enum Direction {
        UP,
        DOWN
    }

    @Override
    public void cancel(){
        robot.kicker.setPower(0);
    }
}
