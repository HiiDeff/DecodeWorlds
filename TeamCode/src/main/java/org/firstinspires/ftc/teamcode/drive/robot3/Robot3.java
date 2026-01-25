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
    public static double PIVOT_CLOSE = 0.06, PIVOT_MID = 0.32, PIVOT_FAR = 0.44; //all the way down is 0.06, all the way up is 0.49
    public static double PARK_DOWN = 0.80, PARK_UP = 0.20;
    public static double BLOCKER_BLOCKING = 0.39, BLOCKER_NONBLOCKING = 0.539;

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
            BRAKING_STRENGTH = 0.45, BRAKING_START = 1;
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
            default:
            case DOWN:
                return RAMP_DOWN;
        }
    }
    @Override
    public double getBlockerPosition(BlockerTask.Position position){
        switch (position){
            case CLOSE:
                return BLOCKER_BLOCKING;
            default:
            case OPEN:
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
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_FAR: PIVOT_FAR;
            case MID:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_MID: PIVOT_MID;
            default:
            case CLOSE:
                return pivot == PivotTask.WhichPivot.LEFT? PIVOT_CLOSE: PIVOT_CLOSE;
        }
    }
    @Override
    public double getParkPosition(ParkTask.WhichPark park, ParkTask.Position position){
        switch (position){
            case DOWN:
                return park == ParkTask.WhichPark.LEFT ? PARK_DOWN : PARK_DOWN;
            default:
            case UP:
                return park == ParkTask.WhichPark.LEFT ? PARK_UP : PARK_UP;
        }
    }

    // Flywheel Regressions
    public static double pivotCoefs[] = {
            23.5512716,
            -3.19218652,
            0.18052277,
            -0.00563123979,
            0.000107918828,
            -0.00000132576893,
            1.04845545e-8,
            -5.16391897e-11,
            1.44139306e-13,
            -1.74165127e-16
    };
    public static double rpmCoefs[] = {
            12.62083-50,
            269.78422,
            -11.66927,
            0.277634,
            -0.00367897,
            0.0000265213,
            -9.03248e-8,
            6.23288e-11,
            2.35428e-13
    };

    @Override
    public double calcPivotPosition() {
        double distToGoalInch = Utils.clamp((getVectorToGoal().getMagnitude()), 25, 140);
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
        double distToGoalInch = Utils.clamp((getVectorToGoal().getMagnitude()), 25, 150);
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
}
