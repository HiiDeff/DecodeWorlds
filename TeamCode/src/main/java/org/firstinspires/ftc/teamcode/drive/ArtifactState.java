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

    public static boolean c1, c2, d1, d2; //TODO: Remove from testing

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double dist1 = robot.leftColorSensor.getDistance(DistanceUnit.INCH);
        double dist2 = robot.rightColorSensor.getDistance(DistanceUnit.INCH);
        Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball distance " + FarAuto.ballnum + " " + dist1 + " " + dist2);

        d1 = (dist1 <= MIN_DETECTION_DIST);
        d2 = (dist2 <= MIN_DETECTION_DIST);
        c1 = toHDColor(robot.rightColorSensor.getNormalizedColors());
        c2 = toHDColor(robot.leftColorSensor.getNormalizedColors());

        hasArtifact = toHDColor(robot.rightColorSensor.getNormalizedColors()) || toHDColor(robot.leftColorSensor.getNormalizedColors()) || (Math.min(dist1, dist2) <= MIN_DETECTION_DIST);
    }

    private boolean toHDColor(NormalizedRGBA color1){
        HSV hsv1 = ImageProcessor.ColorToHsv(color1);
        boolean detectsColor1 = (ImageProcessor.isGreen(hsv1))||(ImageProcessor.isPurple(hsv1));
        //Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball detected" + FarAuto.ballnum + " " +detectsColor1 + " " + detectsColor2);
        //Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv" + hsv1.h + " " +hsv1.s + " " + hsv1.v);
        //Log.i("edbug color sensor detected", "Cycle " + FarAuto.cyclenum + " Ball hsv " + hsv2.h + " " +hsv2.s + " " + hsv2.v);

        return detectsColor1;
    }

    public boolean getArtifactState(){
        return hasArtifact;
    }
}
