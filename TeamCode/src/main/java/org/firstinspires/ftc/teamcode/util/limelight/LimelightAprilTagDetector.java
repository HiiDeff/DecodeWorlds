package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Config
public class LimelightAprilTagDetector extends LimelightProcessorBase {

    protected List<LLResultTypes.FiducialResult> fiducialResults;
    private boolean isRedAlliance = false;
    private Pose3D targetPose = null;
    private Pose robotPose = null;
    private AprilTagType motif = null;

    public LimelightAprilTagDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }

    public void setAllianceColor(boolean isRed) {
        isRedAlliance = isRed;
    }

    public double getDistToGoalInches() {
        if(robotPose!=null) {
            return robotPose.distanceFrom(new Pose()) - LLConfig.xOffset;
        }
        return 0.0;
    }

    @Override
    protected void update() {
        fiducialResults = result.getFiducialResults();
        robotPose = null;
        Log.i("edbug", fiducialResults.size()+" ");
        for(LLResultTypes.FiducialResult aTag: fiducialResults) {
            AprilTagType type = AprilTagType.getAprilTagType(aTag.getFiducialId());
            if(type.isMotif()) {
                motif = type;
            } else if(isRedAlliance ^ (type == AprilTagType.BLUE_GOAL)) {
                targetPose = aTag.getTargetPoseCameraSpace();
                Pose3D pose = aTag.getRobotPoseTargetSpace();
                Position pos = pose.getPosition();
                YawPitchRollAngles orientation = pose.getOrientation();

                double x = metersToInches(pos.z);
                double y = metersToInches(pos.x);
                double a = Math.atan(y/(x-LLConfig.xOffset));

                robotPose = new Pose(x, y, a);

                Log.i("edbug robotpose", robotPose.getX()+" "+robotPose.getY()+" "+robotPose.getHeading());
            }
        }
    }

    public Pose3D getTargetPose() {
        return targetPose;
    }

    public Pose getRobotPose() {
        return robotPose;
    }

    public AprilTagType getMotif() {
        return motif;
    }

    private double metersToInches(double distMeters) {
        return 39.370079*distMeters;
    }
}
