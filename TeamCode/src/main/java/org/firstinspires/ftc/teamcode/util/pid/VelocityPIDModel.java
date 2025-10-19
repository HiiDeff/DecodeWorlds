package org.firstinspires.ftc.teamcode.util.pid;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public abstract class VelocityPIDModel {
    public static int MIN_DELTA_T = 50; // to account for encoder noise
    private double minPower;
    private double maxPower;
    private double kP;
    private double kI;
    private double kD;
    private double feedForward;

    protected double lastTarget = Integer.MAX_VALUE;

    public VelocityPIDModel(PIDCoefficients pidCoefficients) {
        this.minPower = pidCoefficients.minPower;
        this.maxPower = pidCoefficients.maxPower;
        this.kP = pidCoefficients.kP;
        this.kI = pidCoefficients.kI;
        this.kD = pidCoefficients.kD;
        this.feedForward = pidCoefficients.feedForward;
    }

    public void updatePID(PIDCoefficients pidCoefficients) {
        this.minPower = pidCoefficients.minPower;
        this.maxPower = pidCoefficients.maxPower;
        this.kP = pidCoefficients.kP;
        this.kI = pidCoefficients.kI;
        this.kD = pidCoefficients.kD;
    }

    private ElapsedTime timer = null;
    private double lastError = 1e9;
    private double dError_dT = 0.0;
    private double integral = 0.0;
    private boolean integralStarted = false;

    private double prevPower = 0.0;

    public double getPower() {
        double error = getError();
        if (timer == null) {
            lastError = error;
            timer = new ElapsedTime();
        }
        double deltaT = timer.milliseconds();
        if(deltaT>=MIN_DELTA_T) {
            timer.reset();
            dError_dT = (error - lastError) / deltaT;
            lastError = error;
            Log.i("edbug dError_dT", ""+dError_dT);
            Log.i("edbug dT", ""+deltaT);
        }

//        if (integralStarted) {
//            integral += error * deltaT;
//        }

        double deltaPower = kP * error + kI * integral + kD * dError_dT;
        double power = deltaPower + prevPower;
        prevPower = power;

        double absolutePower = Math.abs(power);
        if (absolutePower > maxPower) {
            power = power > 0 ? maxPower : -maxPower;
        } else {
//            if (!integralStarted){
//                integralStarted = true;
//            }
            if (absolutePower < minPower && Math.abs(dError_dT) < getStopErrorDerivative() && Math.abs(error) > getStopError()) {
                power = error > 0 ? minPower : -minPower;
            }
        }

        power = power + feedForward;
        return power;
    }

    public boolean isDone() {
        return Math.abs(lastError) < getStopError() && Math.abs(dError_dT) < getStopErrorDerivative();
    }

    public abstract void cancel();
    // Need to be public for PID tuning tools
    public abstract double getError();
    protected abstract double getStopError();
    //maximum error derivative to be deemed at the target velocity:
    protected abstract double getStopErrorDerivative();

}
