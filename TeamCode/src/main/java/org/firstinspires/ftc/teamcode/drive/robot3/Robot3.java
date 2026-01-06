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
    public static double RAMP_UP = 0.38, RAMP_DOWN = 0.42;
    public static VelocityPIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new VelocityPIDCoefficients(0, 1.0,  0.0002, 0.0, 0.0,0.0005);
    public static double PIVOT_CLOSE = 0.14, PIVOT_MID = 0.29, PIVOT_FAR = 0.47; //all the way down is 0.07, all the way up is 0.5
    public static double PARK_DOWN = 0.96, PARK_UP = 0.42;

    //CLOSE is 20 inches
    //MID is 53 inches
    //FAR is 115 inches
    public static double BLOCKER_BLOCKING = 0.6, BLOCKER_NONBLOCKING = 0.4;
    public static double limelightIntakeOffsetInch = -4.3;

    // Flywheel Tuning Vals
    public static double pivotCoef[] = {
            -6.653177,
            0.7294976,
            -0.03392037,
            0.0008750063,
            -0.00001355061,
            1.290464 * Math.pow(10.0, -7.0),
            -7.400309 * Math.pow(10.0, -10.0),
            2.344162 * Math.pow(10.0, -12.0),
            -3.150742 * Math.pow(10.0, -15.0)
    };
    public static double rpmCoef[] = {
            -144032.529,
            17312.5221,
            -880.32487,
            25.271665,
            -0.45127892,
            0.0052033398,
            -0.00003878999,
            1.8057785 * Math.pow(10.0, -7.0),
            -4.7719158 * Math.pow(10.0, -10.0),
            5.4638853 * Math.pow(10.0, -13.0)
    };

    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(12.5)
            .forwardZeroPowerAcceleration(-35)
            .lateralZeroPowerAcceleration(-70)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.0, 0.02, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0.0, 0.04, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.0008,0.6,0.045))
            .centripetalScaling(0.003);

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
            .xVelocity(75)
            .yVelocity(60)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(-1.5)
            .strafePodX(-6.125)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static double T_VALUE_CONSTRAINT = 0.99;
    public static double TIMEOUT_CONSTRAINT = 100;
    public static double BRAKING_STRENGTH = 0.65;
    public static double BRAKING_START = 1;

    public Robot3(HardwareMap hardwareMap) {
        super(hardwareMap, FOLLOWER_CONSTANTS, DRIVE_CONSTANTS,
                new PinpointLocalizer(hardwareMap, PINPOINT_CONSTANTS),
                getPathConstraints()
        );
    }

    private static PathConstraints getPathConstraints() {
        PathConstraints pc = new PathConstraints(T_VALUE_CONSTRAINT, TIMEOUT_CONSTRAINT, BRAKING_STRENGTH, BRAKING_START);
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


    @Override
    public double calcPivotPosition() {
        double distToGoalInch = Utils.clamp(getVectorToGoal().getMagnitude(), 35, 150);
        double pos = 0;
        double pow = 1;
        for (double v : pivotCoef) {
            pos += pow * v;
            pow *= distToGoalInch;
        }
        return pos;
    }

    @Override
    public int calcFlywheelRpm() {
        double distToGoalInch = Utils.clamp(getVectorToGoal().getMagnitude(), 35, 150);
        double rpm = 0;
        double pow = 1;
        for(double v: rpmCoef) {
            rpm += pow *  v;
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
