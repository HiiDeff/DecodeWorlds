//package org.firstinspires.ftc.teamcode.drive;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
//import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;
//
//@Config
//public class ArtifactState {
//    public static double MIN_DETECTION_DIST = 1.6;
//    private final RobotBase robot;
//    private volatile boolean detected = false; //multiple thread access
//
//    public ArtifactState(RobotBase robot){
//        this.robot = robot;
//    }
//
//    public void update(){
//        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
//        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);
//
//        Log.i("edbug sensor dist", dist1+" "+dist2);
//
//        detected = Math.min(dist1, dist2) <= MIN_DETECTION_DIST;
//    }
//
//    public boolean getArtifactState(){
//        return detected;
//    }
//}

package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.defaultauto.far.FarAuto;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

public class ArtifactState {
    public static double MIN_DETECTION_DIST = 1.7;

    private RobotBase robot;

    private boolean hasArtifact = false;

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);
        Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball distance " + FarAuto.ballnum + " " + dist1 + " " + dist2);

        hasArtifact = toHDColor(robot.rightColorSensor.getNormalizedColors(), robot.leftColorSensor.getNormalizedColors()) || (Math.min(dist1, dist2) <= MIN_DETECTION_DIST);
    }

    private boolean toHDColor(NormalizedRGBA color1, NormalizedRGBA color2){
        HSV hsv1 = ImageProcessor.ColorToHsv(color1);
        HSV hsv2 = ImageProcessor.ColorToHsv(color2);
        boolean detectsColor1 = (hsv1.h>110&&hsv1.h<190)||(ImageProcessor.isPurple(hsv1));
        boolean detectsColor2 = (hsv2.h>110&&hsv2.h<190)||(ImageProcessor.isPurple(hsv2));
        Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball detected" + FarAuto.ballnum + " " +detectsColor1 + " " + detectsColor2);
        Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv" + hsv1.h + " " +hsv1.s + " " + hsv1.v);
        Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv " + hsv2.h + " " +hsv2.s + " " + hsv2.v);
        if(detectsColor2 || detectsColor1){
            return true;
        }

        return false;
    }

    public boolean getArtifactState(){
        return hasArtifact;
    }
}
