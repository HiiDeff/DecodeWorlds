package org.firstinspires.ftc.teamcode.util.pid;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public abstract class VelocityPIDModel {
    private double minPower;
    private double maxPower;
    private double kP;
    private double kI;
    private double kD;
    private double kV;

    public VelocityPIDModel(VelocityPIDCoefficients pidCoefficients) {
        this.minPower = pidCoefficients.minPower;
        this.maxPower = pidCoefficients.maxPower;
        this.kP = pidCoefficients.kP;
        this.kI = pidCoefficients.kI;
        this.kD = pidCoefficients.kD;
        this.kV = pidCoefficients.kV;
    }

    public void updatePID(VelocityPIDCoefficients pidCoefficients) {
        this.minPower = pidCoefficients.minPower;
        this.maxPower = pidCoefficients.maxPower;
        this.kP = pidCoefficients.kP;
        this.kI = pidCoefficients.kI;
        this.kD = pidCoefficients.kD;
        this.kV = pidCoefficients.kV;
    }

    private ElapsedTime timer = null;
    private double lastError = 0.0;
    private double integral = 0.0;
    private double dError = 1e9;
    private double lastTarget = Double.NaN;

    public double getPower() {
        double error = getError();
        double target = getTarget();
        if (timer == null) {
            lastError = error;
            timer = new ElapsedTime();
        }
        double deltaT = Math.max(timer.milliseconds(), 1);
        timer.reset();
        double dtSec = deltaT/1000.0;

        dError = error/dtSec;

        if(Double.isNaN(lastTarget)||target!=lastTarget) {
            integral = 0;
        } else {
            integral += error * dtSec;
        }

        double maxIntegral = 1.0/Math.max(kI, 1e-9);
        integral = Math.max(-maxIntegral, Math.min(integral, maxIntegral));

        double feedForward = kV * target;

        double power = kP * error + kI * integral + kD * dError + feedForward;

        if(power>maxPower) power = maxPower;
        if(power<-maxPower) power = -maxPower;

        if(Math.abs(power)<minPower && Math.abs(error)>getStopError()) {
            power = Math.copySign(minPower, power);
        }

        lastError = error;
        lastTarget = target;

        return power;
    }

    public boolean isDone() {
        Log.i("ndbug lastError", lastError+"");
        return Math.abs(lastError) < getStopError();
    }

    public abstract void cancel();
    // Need to be public for PID tuning tools
    public abstract double getError();
    public abstract double getTarget();
    public abstract double getVelocity();
    protected abstract double getStopError();
    //maximum error derivative to be deemed at the target velocity:
    protected abstract double getStopErrorDerivative();

}
