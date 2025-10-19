package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class PivotTask extends TimedTask {

    public enum WhichPivot{
        LEFT,
        RIGHT
    }

    private static final int RPM = 71;
    private static final int MAX_ROTATION_DEGREE = 360;

    private final RobotBase robot;
    private boolean started;
    private final Position position;
    private final WhichPivot pivot;

    public PivotTask(RobotBase robot, WhichPivot pivot, Position position){
        this.robot = robot;
        this.pivot = pivot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if(!started){
            Servo servo = getPivotServo(robot, pivot);
            double currPos = servo.getPosition();
            double targetPos = robot.getPivotTargetPos(pivot, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }

        return false;
    }

    private static Servo getPivotServo(RobotBase robot, WhichPivot pivot) {
        return pivot == WhichPivot.LEFT ? robot.leftPivot : robot.rightPivot;
    }

    public enum Position {
        CLOSE,
        MID,
        FAR
    }
}
