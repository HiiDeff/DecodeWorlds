package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class KickerTask extends TimedTask{
    RobotBase robot;
    KickerPosition position;

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;


    boolean started = false;

    public KickerTask(RobotBase robot, KickerPosition position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.kicker.getPosition();
            double targetPos = robot.getKickerPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.kicker.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    public enum KickerPosition{
        UP,
        DOWN
    }
}
