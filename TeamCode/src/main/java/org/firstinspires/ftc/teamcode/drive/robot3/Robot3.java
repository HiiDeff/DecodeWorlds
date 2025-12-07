package org.firstinspires.ftc.teamcode.drive.robot3;

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
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;

@Config
public class Robot3 extends RobotBase {

    // Constants
    public static double RAMP_UP = 0.37, RAMP_DOWN = 0.42;
    public static VelocityPIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new VelocityPIDCoefficients(0, 1.0,  0.0002, 0.0, 0.0,0.0005);
    public static double PIVOT_CLOSE = 0.14, PIVOT_MID = 0.29, PIVOT_FAR = 0.42; //all the way down is 0.07, all the way up is 0.5

    //CLOSE is 20 inches
    //MID is 53 inches
    //FAR is 115 inches
    public static double BLOCKER_BLOCKING = 0.65, BLOCKER_NONBLOCKING = 0.9;

    // Flywheel Tuning Vals
    public static double pivotCoef[] = {
            -5.425385,
            0.9716465,
            -0.07182935,
            0.00294599,
            -0.00007341627,
            0.00000115468,
            -1.150426 * Math.pow(10.0, -8.0),
            7.03051 * Math.pow(10.0, -11.0),
            -2.400906 * Math.pow(10.0, -13.0),
            3.503716 * Math.pow(10.0, -16.0)
    };
    public static double rpmCoef[] = {
            -6875.27473, //previously -6775.27473
            1568.71241,
            -110.08847,
            4.2639629,
            -0.10028502,
            0.0014919149,
            -0.000014100386,
            8.2002501 * Math.pow(10.0, -8.0),
            -2.674096 * Math.pow(10.0, -10.0),
            3.7399219 * Math.pow(10.0, -13.0)
    };

    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(11.7)
            .forwardZeroPowerAcceleration(-34.5367405423)
            .lateralZeroPowerAcceleration(-65.3199225832)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .translationalPIDFCoefficients(new PIDFCoefficients(0.35, 0.0, 0.02, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.0, 0.04, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.012,0.0,0.001,0.6,0.04))
            .centripetalScaling(0.0006);

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
            .xVelocity(74)
            .yVelocity(62)
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
    public static double BRAKING_STRENGTH = 0.8;
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
    public double calcPivotPosition() {
        double distToGoalInch = Utils.clamp(getVectorToGoal().getMagnitude(), 20, 120);
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
        double distToGoalInch = Utils.clamp(getVectorToGoal().getMagnitude(), 20, 140);
        double rpm = 0;
        double pow = 1;
        for(double v: rpmCoef) {
            rpm += pow *  v;
            pow *= distToGoalInch;
        }
        return (int) rpm;
    }
}
