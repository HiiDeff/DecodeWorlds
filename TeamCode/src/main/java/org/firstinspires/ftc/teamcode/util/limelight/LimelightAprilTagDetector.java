package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Config
public class LimelightAprilTagDetector extends LimelightProcessorBase {

    protected List<LLResultTypes.FiducialResult> fiducialResults;
    private boolean isRedAlliance = false;
    private Position goalPose = null;
    private Pose robotPose = null; // center is limelight
    private AprilTagType motif = null;
    public static double GOAL_OFFSET_TO_ATAG_INCH = -7.0;
    private static ElapsedTime elapsedTime;

    public LimelightAprilTagDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }

    public void setAllianceColor(boolean isRed) {
        isRedAlliance = isRed;
    }

    @Override
    protected void update() {
        if(elapsedTime==null) elapsedTime = new ElapsedTime();
        Log.i("edbug loop times", elapsedTime.milliseconds()+"");
        elapsedTime.reset();
        fiducialResults = result.getFiducialResults();
        robotPose = null;
        Log.i("edbug", fiducialResults.size()+" ");
        for(LLResultTypes.FiducialResult aTag: fiducialResults) {
            AprilTagType type = AprilTagType.getAprilTagType(aTag.getFiducialId());
            if(type.isMotif()) {
                motif = type;
            } else if(isRedAlliance ^ (type == AprilTagType.BLUE_GOAL)) {
                Pose3D targetPose = aTag.getTargetPoseCameraSpace();
                Log.i("edbug raw goal pose", targetPose.getPosition().x+" "+targetPose.getPosition().y+" "+targetPose.getPosition().z);
                goalPose = pointBehindTag(targetPose, inchesToMeters(GOAL_OFFSET_TO_ATAG_INCH));
                Log.i("edbug transformed goal pose", goalPose.x+" "+goalPose.y+" "+goalPose.z);

                double x = metersToInches(-goalPose.z);
                double y = metersToInches(goalPose.x);
                double a = Math.atan2(y,-x);
                double dist = Math.sqrt(x*x+y*y);
                Log.i("edbug limelight dist to goal", ""+Math.sqrt(x*x+y*y));
                if(dist>70) {
                    robotPose = new Pose(x, y, 0); //this was so annoying to debug
                }
            }
        }
    }

    public Position getGoalPose() {
        return goalPose;
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
    private double inchesToMeters(double distInches) { return distInches/39.370079; }

    public static Position pointBehindTag(Pose3D targetPose, double k) {
        Position position = targetPose.getPosition();
        YawPitchRollAngles angles = targetPose.getOrientation();

        double yaw = angles.getYaw(AngleUnit.RADIANS);
        double pitch = angles.getPitch(AngleUnit.RADIANS);
        double roll = angles.getRoll(AngleUnit.RADIANS);

        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);

        // Rotation matrix R = Rz * Ry * Rx
        double[][] R = new double[3][3];
        R[0][0] = cy * cp;
        R[0][1] = cy * sp * sr - sy * cr;
        R[0][2] = cy * sp * cr + sy * sr;

        R[1][0] = sy * cp;
        R[1][1] = sy * sp * sr + cy * cr;
        R[1][2] = sy * sp * cr - cy * sr;

        R[2][0] = -sp;
        R[2][1] = cp * sr;
        R[2][2] = cp * cr;

        // Local offset behind the tag (along negative z-axis)
        double[] localOffset = new double[]{0, 0, -k};

        // Transform
        double xOffset = R[0][0] * localOffset[0] + R[0][1] * localOffset[1] + R[0][2] * localOffset[2];
        double yOffset = R[1][0] * localOffset[0] + R[1][1] * localOffset[1] + R[1][2] * localOffset[2];
        double zOffset = R[2][0] * localOffset[0] + R[2][1] * localOffset[1] + R[2][2] * localOffset[2];

        // Final coordinates
        double xFinal = position.x + xOffset;
        double yFinal = position.y + yOffset;
        double zFinal = position.z + zOffset;

        return new Position(DistanceUnit.INCH, xFinal, yFinal, zFinal, 0);
    }
}
