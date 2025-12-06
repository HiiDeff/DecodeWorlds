package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class RampTask extends TimedTask {
    private final RobotBase robot;
    private final Position position;
    private static final int RPM = 90;
    private static final int MAX_ROTATION_DEGREE = 300;
    private boolean started = false;

    public RampTask(RobotBase robot, Position position){
        this.robot = robot;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if (!started){
            double currPos = robot.ramp.getPosition();
            double targetPos = robot.getRampPosition(position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            robot.ramp.setPosition(targetPos);
            started = true;
        }
        return false;
    }

    @Override
    public void cancel(){

    }

    public enum Position {
        UP,
        DOWN
    }
}
