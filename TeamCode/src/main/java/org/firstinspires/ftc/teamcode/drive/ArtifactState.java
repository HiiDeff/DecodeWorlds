package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

public class ArtifactState {
    public static double MIN_DIST_WITH_ART = 0.7;

    private RobotBase robot;

    private boolean hasArtifact = false;

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);

        if(dist1 < MIN_DIST_WITH_ART || dist2 < MIN_DIST_WITH_ART){
            hasArtifact = toHDColor(robot.rightColorSensor.getNormalizedColors(), robot.leftColorSensor.getNormalizedColors());
        }
    }

    private boolean toHDColor(NormalizedRGBA color1, NormalizedRGBA color2){
        HSV hsv1 = ImageProcessor.ColorToHsv(color1);
        HSV hsv2 = ImageProcessor.ColorToHsv(color2);
        boolean detectsColor1 = (hsv1.h>110&&hsv1.h<190)||(ImageProcessor.isPurple(hsv1));
        boolean detectsColor2 = (hsv2.h>110&&hsv2.h<190)||(ImageProcessor.isPurple(hsv2));
        if(detectsColor2 || detectsColor1){
            return true;
        }

        return false;
    }

    public boolean getArtifactState(){
        return hasArtifact;
    }
}
