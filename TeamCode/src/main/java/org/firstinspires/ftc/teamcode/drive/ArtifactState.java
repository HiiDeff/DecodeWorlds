package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class ArtifactState {
    public static double MIN_DETECTION_DIST = 0.7;

    private final RobotBase robot;

    private boolean detected = false;

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);

        detected = Math.min(dist1, dist2) <= MIN_DETECTION_DIST;
    }

    public boolean getArtifactState(){
        return detected;
    }
}
