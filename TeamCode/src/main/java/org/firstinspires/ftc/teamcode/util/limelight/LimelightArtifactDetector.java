package org.firstinspires.ftc.teamcode.util.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightArtifactDetector extends LimelightProcessorBase {

    public static double ARTIFACT_HEIGHT_INCH = 4.9;
    protected List<Coords> targetCoords;
    protected List<Double> radii;

    public LimelightArtifactDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }
    @Override
    protected void update() {
        if(result==null) {
            targetCoords.clear();
            return;
        }
        double[] pythonResult = result.getPythonOutput();

        targetCoords = new ArrayList<>();
        for(int i=1; i<pythonResult[0]*3+1; i+=3) {
            if(i+2>=pythonResult.length) break;
            Coords anglesToTarget = pixelToAngle(new Coords(pythonResult[i], pythonResult[i+1]));
            targetCoords.add(anglesToRealWorldPos(anglesToTarget));
            radii.add(pythonResult[i+2]);
        }

//        sort();
    }

    private Coords anglesToRealWorldPos(Coords angles) {
        double thetaX = angles.getX();
        double thetaY = angles.getY();

        double thetaWithHorizontal = LLConfig.angleWithHorizontal + thetaY;
        double yDistInch = (LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2) * Math.tan(Math.toRadians(90+thetaWithHorizontal));

        double yzProjectedDist = Math.sqrt(yDistInch*yDistInch + (LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2)*(LLConfig.zOffset-ARTIFACT_HEIGHT_INCH/2));
        double xDistInch = yzProjectedDist * Math.tan(Math.toRadians(thetaX));

        xDistInch += LLConfig.xOffset;
        yDistInch += LLConfig.yOffset;
        double angle = Math.atan2(xDistInch, yDistInch);

        return new Coords(xDistInch, yDistInch, angle);
    }
}
