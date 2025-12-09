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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.defaultauto.far.FarAuto;
import org.firstinspires.ftc.teamcode.util.objectdetector.HSV;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@Config
public class ArtifactState {
    public static double BACK_MIN_DETECTION_DIST = 1.5;
    public static double FRONT_MIN_DETECTION_DIST = 1.68;

    private RobotBase robot;

    private boolean hasArtifact = false;
    private boolean detectedIntake = false;
    private boolean flashingViolet = false;
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


        double frontDist1 = robot.frontColor1.getDistance(DistanceUnit.INCH);
        double frontDist2 = robot.frontColor2.getDistance(DistanceUnit.INCH);

        boolean front1DetectedDistance = (frontDist1 <= FRONT_MIN_DETECTION_DIST);
        boolean front2DetectedDistance = (frontDist2 <= FRONT_MIN_DETECTION_DIST);
        boolean front1DetectedColor = checkColor(robot.frontColor1.getNormalizedColors());
        boolean front2DetectedColor = checkColor(robot.frontColor2.getNormalizedColors());

        detectedIntake = (front1DetectedColor || front2DetectedColor || front1DetectedDistance || front2DetectedDistance);

        Log.e("adbug front sensors", frontDist1 + " " + frontDist2 + " " + front1DetectedColor + " " + front2DetectedColor + " " + detectedIntake);
        Log.e("adbug back sensors", backDist1 + " " + backDist2 + " " + back1DetectedColor + " " + back2DetectedColor + " " + hasArtifact);

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

        if(timer==null) timer = new ElapsedTime();

        boolean flash = (timer.milliseconds() % 500 <= 250); // one every 500 ms


        if (ready){
            if (flash){
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            }else{
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }

            if (!detectedIntake){
                pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
            }
        }else{
            if (flash){
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            }else{
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }

            if (!detectedIntake){
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            }
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
}
