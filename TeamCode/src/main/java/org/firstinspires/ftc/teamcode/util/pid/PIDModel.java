package org.firstinspires.ftc.teamcode.util.pid;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PIDModel {
    private final double minPower;
    private final double maxPower;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double feedForward;
    private final double feedForward2;

    protected double lastTarget = Integer.MAX_VALUE;

    public PIDModel(PIDCoefficients pidCoefficients) {
        this.minPower = pidCoefficients.minPower;
        this.maxPower = pidCoefficients.maxPower;
        this.kP = pidCoefficients.kP;
        this.kI = pidCoefficients.kI;
        this.kD = pidCoefficients.kD;
        this.feedForward = pidCoefficients.feedForward;
        this.feedForward2 = pidCoefficients.feedForward2;
    }

    private ElapsedTime timer = null;
    private double lastError = 1e9;
    private double dError_dT = 0.0;
    private double integral = 0.0;
    private boolean integralStarted = false;

    public double getPower() {
        double error = getError();
        if (timer == null) {
            lastError = error;
            timer = new ElapsedTime();
        }
        double deltaT = timer.milliseconds();
        timer.reset();
        dError_dT = (error - lastError) / deltaT;

//        if (integralStarted) {
//            integral += error * deltaT;
//        }
        double power = kP * error + kI * integral + kD * dError_dT;

        power += feedForward * getFeedForward();
        power += feedForward2 * getFeedForward2();

        double absolutePower = Math.abs(power);
        if (absolutePower > maxPower) {
            power = power > 0 ? maxPower : -maxPower;
        } else {
//            if (!integralStarted){
//                integralStarted = true;
//            }
            if (absolutePower < minPower && Math.abs(error) < getStopError()) { // Math.abs(dError_dT) < getStopErrorDerivative() &&
                //power = error > 0 ? minPower : -minPower;
                power = 0;
            }
        }
        lastError = error;
        return power;
    }

    public boolean isDone() {
        return Math.abs(lastError) < getStopError();// && Math.abs(dError_dT) < getStopErrorDerivative();
    }

    public abstract void cancel();
    // Need to be public for PID tuning tools
    public abstract double getError();
    public abstract double getFeedForward();
    public double getFeedForward2() {
        return 0;
    }
    protected abstract double getStopError();
    //maximum error derivative to be deemed at the target position:
    protected abstract double getStopErrorDerivative();
}
