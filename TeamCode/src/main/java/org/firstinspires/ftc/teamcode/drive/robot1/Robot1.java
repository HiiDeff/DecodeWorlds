package org.firstinspires.ftc.teamcode.drive.robot1;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;

@Config
public class Robot1 extends RobotBase {

    // Constants
    public static double KICKER_UP = -1, KICKER_DOWN = 0.60;
    public static VelocityPIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new VelocityPIDCoefficients(0, 1.0,  0.01, 0.0, 0.0,0.00053);
    public static double PIVOT_CLOSE = 0.59, PIVOT_MID = 0.40, PIVOT_FAR = 0.31;
    public static double BLOCKER_BLOCKING = 0.65, BLOCKER_NONBLOCKING = 0.9;


    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(11.7)
            .forwardZeroPowerAcceleration(-34.336)
            .lateralZeroPowerAcceleration(-69.191)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0.0, 0.025, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.0, 0.04, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0.0,0.0002,0.6,0.045))
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
            .xVelocity(79.75)
            .yVelocity(64.84)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(-1.5)
            .strafePodX(-6.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static double T_VALUE_CONSTRAINT = 0.99;
    public static double TIMEOUT_CONSTRAINT = 100;
    public static double BRAKING_STRENGTH = 0.8;
    public static double BRAKING_START = 1;

    public Robot1(HardwareMap hardwareMap) {
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
    public double getKickerPower(KickerTask.Direction position) {
        switch (position){
            case UP:
                return KICKER_UP;
            default:
            case DOWN:
                return KICKER_DOWN;
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
}
