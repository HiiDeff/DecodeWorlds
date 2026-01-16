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
    public static boolean shootingWhileMoving = true;
    private boolean isRedAlliance = false;
    private AprilTagType motif = null;
    private Pose limelightPose = null;
    public static double ROBOT_VELOCITY_SHOOTING_COMPENSATION_SCALAR = 0.0;
    private Vector vectorToGoal;
    private double rawDistToGoal;
    public static int MIN_DIST_DETECTABLE = 60; //don't change this

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
    protected void updateInternal() {
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
        //flight time
        //60 inch: 1.0 sec
        //100 inch: 1.2 sec
        //120 inch: 1.3 sec
        // y-1 = 0.005(x-60)
        // y-1 = 0.005x-0.3
        // y = 0.005x + 0.7
        if(shootingWhileMoving) {
            double flightTimeSec = 0.005*toGoal.getMagnitude()+0.4;
            toGoal = toGoal.minus(velocity.times(flightTimeSec));
        }
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

}
