package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public abstract class LimelightProcessorBase {

    protected final Limelight3A limelight;
    protected final LimelightConfig LLConfig;
    protected LLResult result = null;

    public LimelightProcessorBase(Limelight3A limelight, LimelightConfig LLConfig) {
        this.limelight = limelight;
        this.LLConfig = LLConfig;
    }

    public void update() {
        result = limelight.getLatestResult();
        updateInternal();
//        if(!result.isValid()) {
//            Log.i("edbug", ":(");
//            result = null;
//        }
//        else {
//            Log.i("edbug", "here");
//            update();
//        }
    }
    protected Coords pixelToNormalized(Coords pixel) {
        double cx = (LLConfig.resolutionX-1)/2.0;
        double cy = (LLConfig.resolutionY-1)/2.0;

        double nx = (pixel.x-cx) / (LLConfig.resolutionX/2.0);
        double ny = (cy-pixel.y) / (LLConfig.resolutionY/2.0);

        return new Coords(nx, ny);
    }
    protected Coords pixelToAngleRadians(Coords pixel) {
        Coords n = pixelToNormalized(pixel);
        Log.e("adbug ll pixel normalized", n.toString());
        double nx = n.getX(), ny = n.getY();

        // find angle to plane 1 unit away
        double x = nx * Math.tan(LLConfig.hFovRadians/2.0);
        double y = ny * Math.tan(LLConfig.vFovRadians/2.0);

        double thetaX = Math.atan(x);
        double thetaY = Math.atan(y);

        return new Coords(thetaX, thetaY);
    }

    protected abstract void updateInternal();
}
