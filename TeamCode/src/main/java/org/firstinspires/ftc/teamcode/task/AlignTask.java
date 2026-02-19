package org.firstinspires.ftc.teamcode.task;

import com.pedropathing.geometry.BezierPoint;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class AlignTask extends TimedTask{

    private final RobotBase robot;
    private boolean started = false;
    private int finishTimeMillis;

    public AlignTask (RobotBase robot, int finishTimeMillis){
        this.robot = robot;
        setFinishTimeMillis(finishTimeMillis);
    }

    @Override
    public boolean performInternal() {
        if (!started){
//            robot.holdPoint(new BezierPoint(robot.getPose()), robot.getVectorToGoal().getTheta(), false);
            started = true;
        }
        return false;
    }
}
