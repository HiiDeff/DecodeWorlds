package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class PivotTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final RobotBase robot;

    private boolean started;
    private final PivotPosition position;

    public PivotTask(RobotBase robot, PivotPosition position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if(!started){
            double currPos = robot.pivotLeft.getPosition();
            double targetPos = robot.getPivotPosition(position);

            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));

            robot.setPivotPosition(position);

            started = true;
        }

        return false;
    }

    public enum PivotPosition{
        CLOSE,
        MID,
        FAR
    }
}
