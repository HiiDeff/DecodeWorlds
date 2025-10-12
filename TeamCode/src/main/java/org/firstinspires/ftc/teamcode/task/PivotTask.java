package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class PivotTask extends TimedTask {

    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;

    private final RobotBase robot;

    private boolean started;
    private final double position;

    public PivotTask(RobotBase robot, double position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if(!started){
            double currPos = robot.getPivotPosition();
            double targetPos = position;

            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));

            robot.setPivotPosition(targetPos);

            started = true;
        }

        return false;
    }
}
