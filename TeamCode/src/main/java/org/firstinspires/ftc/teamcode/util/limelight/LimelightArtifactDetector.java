package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
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
}
