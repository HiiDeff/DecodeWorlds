package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class ArtifactState {
    public static double MIN_DETECTION_DIST = 1.8;
    private final RobotBase robot;
    private volatile boolean detected = false; //multiple thread access

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);

        Log.i("edbug sensor dist", dist1+" "+dist2);

        detected = Math.min(dist1, dist2) <= MIN_DETECTION_DIST;
    }

    public boolean getArtifactState(){
        return detected;
    }
}
