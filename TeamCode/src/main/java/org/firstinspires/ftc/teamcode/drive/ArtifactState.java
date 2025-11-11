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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.defaultauto.far.FarAuto;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class ArtifactState {
    public static double MIN_DETECTION_DIST = 1.5;

    private RobotBase robot;

    private boolean hasArtifact = false;
    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);

        boolean leftDistance = (dist1 <= MIN_DETECTION_DIST);
        boolean rightDistance = (dist2 <= MIN_DETECTION_DIST);
        boolean leftColor = checkColor(robot.leftColorSensor.getNormalizedColors());
        boolean rightColor = checkColor(robot.rightColorSensor.getNormalizedColors());

        hasArtifact = (leftDistance || rightDistance || leftColor || rightColor);

        Log.i("adebug color sensor distance", "left distance " + dist1 + "\tright distance " + dist2 + "\t\tleft color detected " + leftColor + "\tright color detected " + rightColor);
    }

    private boolean checkColor(NormalizedRGBA color){
        HSV hsv = ImageProcessor.ColorToHsv(color);
        //Log.i("edbug color sensor color", "Cycle " + FarAuto.cyclenum + " Ball detected" + FarAuto.ballnum + " " +detectsColor1 + " " + detectsColor2);
        //Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv" + hsv1.h + " " +hsv1.s + " " + hsv1.v);
        //Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv " + hsv2.h + " " +hsv2.s + " " + hsv2.v);
        return (ImageProcessor.isGreen(hsv))||(ImageProcessor.isPurple(hsv));
    }

    public boolean getArtifactState(){
        return hasArtifact;
    }
}
