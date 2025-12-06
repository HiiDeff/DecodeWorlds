package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.PIDModel;

public class Turret extends PIDModel {


    private final RobotBase robot;
    private final double ticksPerRadian;
    private int targetAngle;
    public Turret(RobotBase robot, PIDCoefficients pidCoefficients, double ticksPerRadian) {
        super(pidCoefficients);
        this.robot = robot;
        this.ticksPerRadian = ticksPerRadian;
    }
    public void setTargetAngle(int targetAngleTicks) {
        targetAngle = targetAngleTicks;
    }

    @Override
    public void cancel() {
        //do nothing
    }

    @Override
    public double getError() {
        int error = targetAngle - robot.getTurretAngleTicks();
        return error;
        //return Math.sqrt(Math.abs(error))*Math.copySign(1, error);
    }

    @Override
    protected double getStopError() {
        return 1;
        //return 0; //don't stop
    }

    @Override
    protected double getStopErrorDerivative() {
        return 0;
    }
}
