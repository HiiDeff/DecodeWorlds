package org.firstinspires.ftc.teamcode.util.limelight;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.List;

@Config
public class LimelightAprilTagDetector extends LimelightProcessorBase {

    // if limelight seen in the last update loop and there's a large jump in the angle of the apriltag, then return null

    public static Pose RED_GOAL_POSE = new Pose(-65, 65);
    public static Pose BLUE_GOAL_POSE = new Pose(-65, -65);

    private boolean isRedAlliance = false;
    private AprilTagType motif = null;
    private Pose limelightPose = null;
    public static double ROBOT_VELOCITY_SHOOTING_COMPENSATION_SCALAR = 0.7;
    private Vector vectorToGoal;
    private double rawDistToGoal;
    public static int MIN_DIST_DETECTABLE = 60; //don't change this

    private boolean goalSeenLastLoop = false;
    private ElapsedTime timeSinceLastDetection = null;
    private double lastSeenGoalYaw = 0.0;
    public static int SUDDEN_ATAG_JUMP_IN_YAW_FILTER_MS = 500, SUDDEN_ATAG_JUMP_IN_YAW_FILTER_DEGREE = 10;

    public LimelightAprilTagDetector(Limelight3A limelight, LimelightConfig LLConfig) {
        super(limelight, LLConfig);
    }

    public void setAllianceColor(boolean isRed) {
        isRedAlliance = isRed;
    }

    @Override
    protected void update() {
        if(timeSinceLastDetection==null) {
            timeSinceLastDetection = new ElapsedTime();
        }
        limelightPose = null;
        if(rawDistToGoal<MIN_DIST_DETECTABLE) return;
        for(LLResultTypes.FiducialResult aTag: result.getFiducialResults()) {
            AprilTagType type = AprilTagType.getAprilTagType(aTag.getFiducialId());
            if(type.isMotif()) {
                motif = type;
            } else if(isRedAlliance ^ (type == AprilTagType.BLUE_GOAL)) {
                YawPitchRollAngles llAngles = aTag.getRobotPoseFieldSpace().getOrientation();
                Position llPos = aTag.getRobotPoseFieldSpace().getPosition();
                //Log.i("edbug limelight pos", llAngles.getYaw()+" "+llAngles.getPitch()+" "+llAngles.getRoll());
                Log.i("edbug limelight pos", metersToInches(llPos.x)+" "+metersToInches(llPos.y)+" "+metersToInches(llPos.z));
                Log.i("edbug atag yaw", llAngles.getYaw(AngleUnit.DEGREES)+"");
                Log.i("edbug goal seen last loop", goalSeenLastLoop+"");

                if(!(timeSinceLastDetection.milliseconds()<SUDDEN_ATAG_JUMP_IN_YAW_FILTER_MS && Math.abs(llAngles.getYaw(AngleUnit.DEGREES)-lastSeenGoalYaw) > SUDDEN_ATAG_JUMP_IN_YAW_FILTER_DEGREE)) {
                    limelightPose = new Pose(metersToInches(llPos.x), metersToInches(llPos.y), llAngles.getYaw(AngleUnit.RADIANS));
                    lastSeenGoalYaw = llAngles.getYaw(AngleUnit.DEGREES);
                }
                timeSinceLastDetection.reset();
            }
        }
    }
    public void updateVectorToGoal(Pose pose, Vector velocity) {
        Pose goalPose = isRedAlliance ? RED_GOAL_POSE : BLUE_GOAL_POSE;
        Vector toGoal = new Vector(goalPose.minus(pose));
        rawDistToGoal = toGoal.getMagnitude();
        toGoal = toGoal.minus(velocity.times(ROBOT_VELOCITY_SHOOTING_COMPENSATION_SCALAR));
        toGoal.setTheta(Utils.normalize(toGoal.getTheta()));
        vectorToGoal = toGoal;
    }
    public Vector getVectorToGoal() {
        return vectorToGoal;
    }
    public double getRawDistToGoal() {
        return rawDistToGoal;
    }

    public Pose getLimelightFieldPose() {
        return limelightPose;
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
