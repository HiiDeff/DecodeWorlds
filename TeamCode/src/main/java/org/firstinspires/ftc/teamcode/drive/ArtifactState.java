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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class ArtifactState {
    public static double BACK_MIN_DETECTION_DIST = 1.0;
    public static double FRONT_MIN_DETECTION_DIST = 1.52;

    public static int CURRENT_2_BALL_THRESHOLD = 1000;

    private RobotBase robot;

    private double intakeCurrent = 0.0;
    private boolean hasArtifact = false;
    private boolean detectedIntake = false;
    private boolean flashingViolet = false;
    private int ballCount = 0;
    private ElapsedTime timer = null;

    public ArtifactState(RobotBase robot){
        this.robot = robot;
    }

    public void update(){
        double backDist1 = robot.backColor1.getDistance(DistanceUnit.INCH);
        double backDist2 = robot.backColor2.getDistance(DistanceUnit.INCH);

        boolean back1DetectedDistance = (backDist1 <= BACK_MIN_DETECTION_DIST);
        boolean back2DetectedDistance = (backDist2 <= BACK_MIN_DETECTION_DIST);
        boolean back1DetectedColor = checkColor(robot.backColor1.getNormalizedColors());
        boolean back2DetectedColor = checkColor(robot.backColor2.getNormalizedColors());

        hasArtifact = (back1DetectedDistance || back2DetectedDistance || back1DetectedColor || back2DetectedColor);

        detectedIntake = !robot.breakBeamReceiver.getState();

        intakeCurrent = robot.intake.getCurrent(CurrentUnit.MILLIAMPS);

        if(detectedIntake && hasArtifact){
            ballCount = 3;
        }
        else if(hasArtifact){
            if(intakeCurrent >= CURRENT_2_BALL_THRESHOLD){
                ballCount = 2;
            }
            else{
                ballCount = 1;
            }
        }
        else{
            ballCount = 0;
        }

        Log.e("adbug break beam", detectedIntake + "");
        Log.e("adbug back sensors", backDist1 + " " + backDist2 + " " + back1DetectedColor + " " + back2DetectedColor + " " + hasArtifact);
        Log.i("ndbug ball count", ballCount + "");
//        HSV hsv1 = ImageProcessor.ColorToHsv(robot.frontColor1.getNormalizedColors());
//        HSV hsv2 = ImageProcessor.ColorToHsv(robot.frontColor2.getNormalizedColors());
//
//        Log.e("adbug frontcolor1", hsv1.h + " " + hsv1.s + " " + hsv1.v);
//        Log.e("adbug frontcolor2", hsv2.h + " " + hsv2.s + " " + hsv2.v);


        setStripLightColors();
    }

    private void setStripLightColors() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

        boolean ready = robot.flywheelAtTarget() && robot.turretAtTarget();

        if (ready){
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }else{
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }


        Log.e("adbug colorled",  pattern.toString());

        robot.setLightColor(pattern);
    }

    private boolean checkColor(NormalizedRGBA color){
        HSV hsv = ImageProcessor.ColorToHsv(color);
        return (ImageProcessor.isGreen(hsv))||(ImageProcessor.isPurple(hsv));
    }

    public boolean getArtifactState(){
        return hasArtifact;
    }

    public int getBallCount(){
        return ballCount;
    }
}
