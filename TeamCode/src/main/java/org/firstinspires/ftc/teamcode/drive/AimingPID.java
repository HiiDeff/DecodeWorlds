package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

public class AimingPID extends PIDModel {

    private final RobotBase robot;

    public AimingPID(RobotBase robot, PIDCoefficients pidCoefficients) {
        super(pidCoefficients);
        this.robot = robot;
    }

    public void cancel(){

    }
    // Need to be public for PID tuning tools
    public double getError(){
        Pose current = this.robot.getPose();

        double currentAngle = this.robot.getHeading();

        double targetAngle = Math.PI + Math.atan(current.getY() - current.getX());

        return targetAngle - currentAngle;

    }
    protected double getStopError(){

    }
    //maximum error derivative to be deemed at the target position:
    protected double getStopErrorDerivative(){

    }
}
