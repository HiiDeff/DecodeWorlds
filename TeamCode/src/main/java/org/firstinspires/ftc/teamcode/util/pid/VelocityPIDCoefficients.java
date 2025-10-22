package org.firstinspires.ftc.teamcode.util.pid;

public class VelocityPIDCoefficients extends PIDCoefficients {
    public double kV;
    public VelocityPIDCoefficients(double minPower, double maxPower, double kP, double kI, double kD, double kV) {
        super(minPower, maxPower, kP, kI, kD);
        this.kV = kV;
    }
}