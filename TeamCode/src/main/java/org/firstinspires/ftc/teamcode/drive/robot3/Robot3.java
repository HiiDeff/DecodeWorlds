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
    public static double RAMP_UP = 0.54, RAMP_DOWN = 0.49;
    public static VelocityPIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new VelocityPIDCoefficients(0, 1.0,  0.00001, 0.0, 0.0,0);
    public static double PIVOT_CLOSE = 0.2, PIVOT_MID = 0.29, PIVOT_FAR = 0.47; //all the way down is 0.07, all the way up is 0.5
    public static double PARK_DOWN = 0.96, PARK_UP = 0.42;

    //CLOSE is 20 inches
    //MID is 53 inches
    //FAR is 115 inches
    public static double BLOCKER_BLOCKING = 0.36, BLOCKER_NONBLOCKING = 0.539;
    public static double limelightIntakeOffsetInch = -4.3;

    // Flywheel Tuning Vals
    public static double pivotCoef[] = {
            1.1525,
            -0.100701,
            0.0039865,
            -0.0000834899,
            0.00000113928,
            -1.08136* Math.pow(10.0, -8.0),
            6.86432 * Math.pow(10.0, -11.0),
            -2.5508 * Math.pow(10.0, -13.0),
            4.09469 * Math.pow(10.0, -16.0)
    };
    public static double rpmCoef[] = {
            -36728.6253,
            5020.70529,
            -277.62134,
            8.6969233,
            -0.17003434,
            0.0021556971,
            -0.000017750227,
            9.164225 * Math.pow(10.0, -8.0),
            -2.6939193 * Math.pow(10.0, -10.0),
            3.4370523 * Math.pow(10.0, -13.0)
    };

    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(12.6)
            .forwardZeroPowerAcceleration(-35.618068367118305)
            .lateralZeroPowerAcceleration(-68)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .useSecondaryHeadingPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.0, 0.5, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.5, 0.0, 0.0, 0.003))
            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0.0, 0.04, 0.020))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4, 0.0, 0.04, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0.0,0.011,0.6,0.04))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.00008,0.6,0.04))
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

    public static double T_VALUE_CONSTRAINT = 0.99;
    public static double TIMEOUT_CONSTRAINT = 100;
    public static double BRAKING_STRENGTH = 1.1;
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
