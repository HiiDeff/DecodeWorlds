package org.firstinspires.ftc.teamcode.task;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.util.Utils;

public class ParkTask extends TimedTask {

    public enum WhichPark{
        LEFT,
        RIGHT
    }

    private static final int RPM = 71;
    private static final int MAX_ROTATION_DEGREE = 360;

    private final RobotBase robot;
    private boolean started;
    private final Position position;
    private final WhichPark park;

    public ParkTask(RobotBase robot, WhichPark park, Position position){
        this.robot = robot;
        this.park = park;
        this.position = position;
    }

    @Override
    public boolean performInternal(){
        if(!started){
            Servo servo = getParkServo(robot, park);
            double currPos = servo.getPosition();
            double targetPos = robot.getParkPosition(park, position);
            setFinishTimeMillis(Utils.getEstimatedServoTime(RPM, MAX_ROTATION_DEGREE,
                    targetPos - currPos));
            servo.setPosition(targetPos);
            started = true;
        }

        return false;
    }

    private static Servo getParkServo(RobotBase robot, WhichPark park) {
        return park == WhichPark.LEFT ? robot.leftPark : robot.rightPark;
    }

    public enum Position {
        DOWN,
        UP
    }
}
