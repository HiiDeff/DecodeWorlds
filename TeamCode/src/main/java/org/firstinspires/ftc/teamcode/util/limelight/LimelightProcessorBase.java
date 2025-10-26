package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.drive.RobotBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public abstract class LimelightProcessorBase {

    protected final Limelight3A limelight;
    protected final LimelightConfig LLConfig;
    protected LLResult result = null;

    public LimelightProcessorBase(Limelight3A limelight, LimelightConfig LLConfig) {
        this.limelight = limelight;
        this.LLConfig = LLConfig;
    }

    public void updateLimelight() {
        result = limelight.getLatestResult();
        update();
//        if(!result.isValid()) {
//            Log.i("edbug", ":(");
//            result = null;
//        }
//        else {
//            Log.i("edbug", "here");
//            update();
//        }
    }

    // please just don't
    public synchronized LLResult getLLResult() {
        return result;
    }

    protected double toRadians(double angDeg) {
        return Math.toRadians(angDeg);
    }

    protected double toDegrees(double angRad) {
        return Math.toDegrees(angRad);
    }

    protected abstract void update();
}
