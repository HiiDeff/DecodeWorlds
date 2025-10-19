package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDModel;

public class FlywheelPID extends VelocityPIDModel {

    private final RobotBase robot;
    private double targetVelocity;

    public FlywheelPID(RobotBase robot, PIDCoefficients pidCoefficients) {
        super(pidCoefficients);
        this.robot = robot;
    }

    public void setTargetVelocity(double targetVelocityTicksPerSecond) {
        targetVelocity = targetVelocityTicksPerSecond;
    }

    @Override
    public void cancel() {
        //what to do if cancel? probably just holding power but don't use cancel()
    }

    @Override
    public double getError() {
        return targetVelocity - robot.getFlywheelVelocityTicksPerSecond(); //return error of velocity
    }

    @Override
    protected double getStopError() {
        return 5;
    } //minimum error to stop adjusting

    @Override
    protected double getStopErrorDerivative() {
        return 1; //0.001 ticks/ms = 1 ticks/sec
    } //minimum acceleration to stop adjusting (ticks per ms^2)
}
