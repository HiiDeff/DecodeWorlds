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
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;

@Config
public class Robot1 extends RobotBase {

    // Constants
    public static double KICKER_UP = 0.65, KICKER_DOWN = 0.9;

    public static PIDCoefficients FLYWHEEL_VELOCITY_PID_COEFFICIENTS = new PIDCoefficients(0, 1.0,  0.00001, 0.0, 0.001,0);

    public static double LEFT_PIVOT_CLOSE = 0.0, LEFT_PIVOT_MID = 0.0, LEFT_PIVOT_FAR = 0.0;
    public static double RIGHT_PIVOT_CLOSE = 0.0, RIGHT_PIVOT_MID = 0.0, RIGHT_PIVOT_FAR = 0.0;

    // Pedro Constants
    public static FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(10.65)
            .forwardZeroPowerAcceleration(-34.85468883462399)
            .lateralZeroPowerAcceleration(-60.980555558410465)
            .useSecondaryDrivePIDF(false)/*true for 2 PIDs*/
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0.0, 0.020, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(1.7, 0.0, 0.06, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0.0,0.0002,0.6,0.048));

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
            .xVelocity(77.70272226408711)
            .yVelocity(57.90664264348548);

    public static PinpointConstants PINPOINT_CONSTANTS = new PinpointConstants()
            .forwardPodY(-7.125)
            .strafePodX(3)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
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
    public double getKickerPosition(KickerTask.Position position) {
        switch (position){
            case UP:
                return KICKER_UP;
            default:
            case DOWN:
                return KICKER_DOWN;
        }
    }

    @Override
    public PIDCoefficients getVelocityPIDCoefficients() {
        return FLYWHEEL_VELOCITY_PID_COEFFICIENTS;
    }

    @Override
    public double getPivotTargetPos(PivotTask.WhichPivot pivot, PivotTask.Position position) {
        switch (position){
            case FAR:
                return pivot == PivotTask.WhichPivot.LEFT? LEFT_PIVOT_FAR:RIGHT_PIVOT_FAR;
            case MID:
                return pivot == PivotTask.WhichPivot.LEFT? LEFT_PIVOT_MID:RIGHT_PIVOT_MID;
            default:
            case CLOSE:
                return pivot == PivotTask.WhichPivot.LEFT? LEFT_PIVOT_CLOSE:RIGHT_PIVOT_CLOSE;
        }
    }
}
