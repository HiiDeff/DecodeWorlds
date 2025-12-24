package org.firstinspires.ftc.teamcode.util.limelight;


public class LimelightConfig {
    // all measurements are in inches and degrees
    public int resolutionX, resolutionY;
    public double hFovRadians, vFovRadians;
    public double angleWithHorizontal;
    public double xOffset, yOffset, zOffset;
    // xOffset: right is positive, left is negative
    // yOffset: forwards is positive, backwards is negative
    // zOffset: up is positive, down is negative
    // offsets are relative to the tracking point of road runner, which is usually the center of the robot.

    public double errorScalerX, errorScalerY;
    // Tune such that the position is accurate to the (0, 12, 0) relative to the limelight at (0, 0, z), then calculate the error / distance for different values of x and y

    // When all the values are correct, everything should be very accurate, only the zOffset (height of the camera) should be used to tune the yOffset

    public LimelightConfig(int resolutionX, int resolutionY,
                           double angleWithHorizontalDegrees,
                           double horizontalFOVDegrees, double verticalFOVDegrees,
                           double xOffsetInch, double yOffsetInch, double zOffsetInch) {
        this.resolutionX = resolutionX;
        this.resolutionY = resolutionY;
        this.hFovRadians = Math.toRadians(horizontalFOVDegrees);
        this.vFovRadians = Math.toRadians(verticalFOVDegrees);
        this.angleWithHorizontal = angleWithHorizontalDegrees;
        this.xOffset = xOffsetInch;
        this.yOffset = yOffsetInch;
        this.zOffset = zOffsetInch;
    }
    public LimelightConfig(int resolutionX, int resolutionY,
                           double angleWithHorizontalDegrees,
                           double horizontalFOV, double verticalFOV,
                           double xOffsetInch, double yOffsetInch, double zOffsetInch,
                           double errorScalerX, double errorScalerY) {
        this(resolutionX, resolutionY, angleWithHorizontalDegrees, horizontalFOV, verticalFOV, xOffsetInch, yOffsetInch, zOffsetInch);
        this.errorScalerX = errorScalerX;
        this.errorScalerY = errorScalerY;
    }
}