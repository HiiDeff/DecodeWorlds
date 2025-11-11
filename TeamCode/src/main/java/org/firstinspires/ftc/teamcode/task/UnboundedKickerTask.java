package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class UnboundedKickerTask implements Task, UnboundedTask {
    private final RobotBase robot;
    private final KickerTask.Direction direction;
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private boolean started = false;

    public UnboundedKickerTask(RobotBase robot, KickerTask.Direction direction){
        this.robot = robot;
        this.direction = direction;
    }

    @Override
    public boolean perform(){
        if (!started){
            robot.kicker.setPower(robot.getKickerPower(direction));
            started = true;
        }
        return false;
    }

    @Override
    public void cancel(){
        robot.kicker.setPower(0);
    }
}
