package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDModel;

public class FlywheelPID extends VelocityPIDModel {

    private final RobotBase robot;
    private double targetVelocity;
    public static int pow = 3;

    public FlywheelPID(RobotBase robot, VelocityPIDCoefficients pidCoefficients) {
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
        double error = targetVelocity - getVelocity();
        if(error>0) {
            return Math.pow(error, pow);
        }
        return error;
    }

    @Override
    public double getTarget() {
        return targetVelocity;
    }

    @Override
    public double getVelocity() {
        return robot.getFlywheelVelocityTicksPerSecond();
    }

    @Override
    protected double getStopError() {
        // minimum error to stop adjusting, in ticks per second
        // ticks per second for 100 RPM
        double errorBoundRpm = 100;
        return Math.pow(errorBoundRpm*28/60, 2);
    }

    @Override
    protected double getStopErrorDerivative() {
        //minimum dError_dT to stop adjusting (ticks per ms^2)
        return 0.2;
    }
}
