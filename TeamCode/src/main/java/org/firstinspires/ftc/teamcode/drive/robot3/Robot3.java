package org.firstinspires.ftc.teamcode.drive.robot3;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.ParkTask;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.Coords;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;

import java.util.ArrayList;
import java.util.List;

@Config
public class Robot3 extends RobotBase {

    // Constants
    public static double RAMP_UP = 0.55, RAMP_DOWN = 0.49;
    public static VelocityPIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new VelocityPIDCoefficients(0, 1.0,  0.150, 0.0, 0.0,0.00038);
    public static double PIVOT_CLOSE = 0.06, PIVOT_MID = 0.36, PIVOT_FAR = 0.47, PIVOT_SORT = 0.47; //all the way down is 0.06, all the way up is 0.49
    public static double PARK_DOWN = 0.80, PARK_UP = 0.20;
    public static double BLOCKER_BLOCKING = 0.39, BLOCKER_NONBLOCKING = 0.539;
    public static int GATE_LOADING_ZONE_CUTOFF_X = 36;

    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(12.6)
            .forwardZeroPowerAcceleration(-35.618068367118305)//-30.247
            .lateralZeroPowerAcceleration(-68)//-65.309
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .useSecondaryHeadingPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.0, 0.02, 0.02))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.5, 0.0, 0.0, 0.003))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.1, 0.020))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4, 0.0, 0.04, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,0.0,0.0025,0.6,0.04))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.00008,0.6,0.04))
            .centripetalScaling(0.0005);

    public static MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .maxPower(1.0)
            .xVelocity(79.19765861858123)
            .yVelocity(61.66185161427456)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(-6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static double T_VALUE_CONSTRAINT = 0.99,
            VELOCITY_CONSTRAINT = 0.1, TRANSLATIONAL_CONSTRAINT = 0.1,
            HEADING_CONSTRAINT = 0.007, TIMEOUT_CONSTRAINT = 100,
            BRAKING_STRENGTH = 1, BRAKING_START = 1;
    public static int BEZIER_CURVE_SEARCH_LIMIT = 10;

    public Robot3(HardwareMap hardwareMap) {
        super(hardwareMap, FOLLOWER_CONSTANTS, DRIVE_CONSTANTS,
                new PinpointLocalizer(hardwareMap, PINPOINT_CONSTANTS),
                getPathConstraints()
        );
    }

    private static PathConstraints getPathConstraints() {
        PathConstraints pc = new PathConstraints(T_VALUE_CONSTRAINT, VELOCITY_CONSTRAINT, TRANSLATIONAL_CONSTRAINT, HEADING_CONSTRAINT, TIMEOUT_CONSTRAINT, BRAKING_STRENGTH, BEZIER_CURVE_SEARCH_LIMIT, BRAKING_START);
        PathConstraints.setDefaultConstraints(pc);
        return pc;
    }

    @Override
    public double getRampPosition(RampTask.Position position) {
        switch (position){
            case UP:
                return RAMP_UP;
            case DOWN:
            default:
                return RAMP_DOWN;
        }
    }
    @Override
    public double getBlockerPosition(BlockerTask.Position position){
        switch (position){
            case CLOSE:
                return BLOCKER_BLOCKING;
            case OPEN:
            default:
                return BLOCKER_NONBLOCKING;
        }
    }
    @Override
    public VelocityPIDCoefficients getVelocityPIDCoefficients() {
        return FLYWHEEL_VELOCITY_PID_COEFFICIENTS;
    }
    @Override
    public double getPivotTargetPos(PivotTask.WhichPivot pivot, PivotTask.Position position) {
        switch (position){
            case FAR:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_FAR:PIVOT_FAR;
            case MID:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_MID:PIVOT_MID;
            case SORT:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_SORT:PIVOT_SORT;
            case CLOSE:
            default:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_CLOSE:PIVOT_CLOSE;
        }
    }
    @Override
    public double getParkPosition(ParkTask.WhichPark park, ParkTask.Position position){
        switch (position){
            case DOWN:
                return park == ParkTask.WhichPark.LEFT ? PARK_DOWN : PARK_DOWN;
            case UP:
            default:
                return park == ParkTask.WhichPark.LEFT ? PARK_UP : PARK_UP;
        }
    }

    // Flywheel Regressions
    public static double pivotCoefs[] = {
            -155.735128,
            19.9331173,
            -1.09734315,
            0.034113584,
            -0.000660471248,
            0.00000827346758,
            -6.71946791e-8,
            3.41913343e-10,
            -9.91057983e-13,
            1.2490502e-15
    };
    public static double rpmCoefs[] = {
            -28963.7071,
            1988.3742,
            32.024883,
            -6.6183559,
            0.2715061,
            -0.0058721863,
            0.000077233521,
            -6.3892117e-7,
            3.2541566e-9,
            -9.3350318e-12,
            1.1550232e-14
    };

    @Override
    public double calcPivotPosition() {
        double distToGoalInch = Utils.clamp((getVectorToGoal().getMagnitude()), 35, 140);
        if(distToGoalInch>130) { //linear interpolation
            return 0.001*(distToGoalInch-130) + 0.46;
        }
        double pos = 0;
        double pow = 1;
        for (double v : pivotCoefs) {
            pos += pow * v;
            pow *= distToGoalInch;
        }
        return pos;
    }

    @Override
    public int calcFlywheelRpm() {
        double distToGoalInch = Utils.clamp((getVectorToGoal().getMagnitude()), 35, 150);
        if(distToGoalInch>140) { //linear interpolation
            return (int) (15.0*(distToGoalInch-140) + 4150);
        }
        double rpm = 0;
        double pow = 1;
        for(double v: rpmCoefs) {
            rpm += pow * v;
            pow *= distToGoalInch;
        }
        return (int) rpm;
    }


    @Override
    public Pose getTargetArtifactClusterPose(){
        Coords coordsFromLimelight = limelightArtifactDetector.getTargetPosition();
        Coords coordsFromIntake = new Coords(coordsFromLimelight.getX(), coordsFromLimelight.getY(),
                coordsFromLimelight.getZ(), coordsFromLimelight.getAngle());

        return coordsToPose(coordsFromIntake);
    }

    public List<Pose> getTopThreeTargetPositions(){
        List<Coords> coordsFromLimelight = limelightArtifactDetector.getTopThreeTargetPositions();

        List<Pose> result = new ArrayList<>();

        for (Coords location: coordsFromLimelight){
            Coords coordsFromIntake = new Coords(location.getX(), location.getY(),
                    location.getZ(), location.getAngle());

            result.add(coordsToPose(coordsFromIntake));
        }


        return result;
    }

    public Location getArtifactDensestLocation(){
        List<Coords> artifactCoords = limelightArtifactDetector.getArtifactCoords();

        int gateArtifactCount = 0;
        int loadingZoneArtifactCount = 0;

        for (Coords artifact: artifactCoords){
            Pose artifactPose = coordsToPose(artifact);

            // No seeing reflections of artifacts in the field wall
            if (artifactPose.getX() > 72 || artifactPose.getX() < 0 || artifactPose.getY() > 72 || artifactPose.getY() < 0){
                continue;
            }

            if (artifactPose.getX() >= GATE_LOADING_ZONE_CUTOFF_X){
                loadingZoneArtifactCount += 1;
            }else{
                gateArtifactCount += 1;
            }
        }

        if (gateArtifactCount < loadingZoneArtifactCount){
            return Location.GATE;
        }

        return Location.LOADING_ZONE; // Prefer loading zone - balls might roll there
    }
}
