package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class BlockerTask extends TimedTask{

    private final RobotBase robot;
    private final Position position;
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private boolean started = false;

    public BlockerTask(RobotBase robot, Position position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.blocker.getPosition();
            double targetPos = robot.getBlockerPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));

            robot.blocker.setPosition(targetPos);

            started = true;
        }

        return false;
    }

    public enum Position{
        CLOSE,
        OPEN
    }


}
