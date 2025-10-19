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
        // do nothing if cancel (maintain velocity)
    }

    @Override
    public double getError() {
        return targetVelocity - robot.getFlywheelVelocityTicksPerSecond();
    }

    @Override
    protected double getStopError() {
        // minimum error to stop adjusting, in ticks per second
        // ticks per second for 70 RPM
        return 70.0*28/60;
    }

    @Override
    protected double getStopErrorDerivative() {
        //minimum dError_dT to stop adjusting (ticks per ms^2)
        return 0.2;
    }
}
