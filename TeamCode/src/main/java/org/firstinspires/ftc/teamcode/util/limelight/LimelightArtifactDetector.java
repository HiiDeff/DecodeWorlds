package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.auto.Location;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class LimelightArtifactDetector extends LimelightProcessorBase {

    public static double ARTIFACT_HEIGHT_INCH = 4.9;
    protected List<Coords> targetCoords = new ArrayList<>();
    protected List<Double> radii = new ArrayList<>();

    public LimelightArtifactDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }
    @Override
    protected void updateInternal() {
        targetCoords.clear();
        radii.clear();
        if(result==null) {
            return;
        }
        double[] pythonResult = result.getPythonOutput();
        for(int i=1; i<pythonResult[0]*3+1; i+=3) {
            if(i+2>=pythonResult.length) break;
            Log.e("adbug ll python output", "index " + i + "\t" + pythonResult[i] + " " + pythonResult[i+1] + " " + pythonResult[i+2]);
            Coords anglesToTarget = pixelToAngleRadians(new Coords(pythonResult[i], pythonResult[i+1]));
            Log.e("adbug ll angle to target", anglesToTarget.toString());
            targetCoords.add(anglesToRealWorldPos(anglesToTarget));
            radii.add(pythonResult[i+2]);

            Log.e("adbug detected artifact", "LL Output Index " + i + "\t Target " + targetCoords.get(targetCoords.size() - 1).toString() + " radius " + radii.get(radii.size() - 1));
        }
    }

    public Coords getTargetPosition() {
        if (targetCoords == null || targetCoords.isEmpty()){
            return new Coords(0, 0);
        }

        double maxRadius = 0;
        int idx = 0;
        for(int i=0; i<radii.size(); i++){
            if(radii.get(i)>maxRadius) {
                maxRadius = radii.get(i);
                idx = i;
            }
        }
        return targetCoords.get(idx);
    }

    public List<Coords> getTopThreeTargetPositions() {
        if (targetCoords == null || targetCoords.isEmpty()){
            return null;
        }

        List<Integer> indices = new ArrayList<>();

        for (int i = 0; i < radii.size(); i++){
            indices.add(i);
        }

        // Sort indices by decreasing radii
        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare (Integer a, Integer b){
                double difference = radii.get(b) - radii.get(a);

                if (difference < 0){
                    return -1;
                }else if (difference > 0){
                    return 1;
                }
                return 0;
            }
        });

        return Arrays.asList(targetCoords.get(indices.get(0)), targetCoords.get(indices.get(1)), targetCoords.get(indices.get(2)));

//        double maxRadius1 = 0;
//        double maxRadius2 = 0;
//        double maxRadius3 = 0;
//        int idx1 = 0;
//        int idx2 = 0;
//        int idx3 = 0;
//        for(int i=0; i<radii.size(); i++){
//            if(radii.get(i)>=maxRadius1) {
//                idx3 = idx2;
//                maxRadius3 = maxRadius2;
//
//                idx2 = idx1;
//                maxRadius2 = maxRadius1;
//
//                maxRadius1 = radii.get(i);
//                idx1 = i;
//            }else if (radii.get(i) >= maxRadius2){
//                idx3 = idx2;
//                maxRadius3 = maxRadius2;
//
//                maxRadius2 = radii.get(i);
//                idx2 = i;
//            }else if (radii.get(i) >= maxRadius3){
//                maxRadius3 = radii.get(i);
//                idx3 = i;
//            }
//        }
//        return Arrays.asList(targetCoords.get(idx1), targetCoords.get(idx2), targetCoords.get(idx3));
    }

    private Coords anglesToRealWorldPos(Coords anglesRadians) {
        double thetaX = anglesRadians.getX();
        double thetaY = anglesRadians.getY();

        double thetaWithHorizontal = LLConfig.angleWithHorizontal + thetaY;
        double yDistInch = (LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2) * Math.tan(Math.PI / 2 +thetaWithHorizontal);

        double yzProjectedDist = Math.sqrt(yDistInch*yDistInch + (LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2)*(LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2));

        Log.e("adbug real world conversion", yzProjectedDist + " " + thetaX + " " + yzProjectedDist * Math.tan(thetaX));

        double xDistInch = yzProjectedDist * Math.tan(thetaX);

        xDistInch += LLConfig.xOffset;
        yDistInch += LLConfig.yOffset;
        double angle = Math.atan2(xDistInch, yDistInch);

        return new Coords(xDistInch, yDistInch, angle);
    }

    public List<Coords> getArtifactCoords(){
        return targetCoords;
    }
}
