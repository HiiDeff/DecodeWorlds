package org.firstinspires.ftc.teamcode.drive.robot3;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
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
        super(hardwareMap);

        setDriveParams();

        Log.i("encoder direction", PinpointDrive.PARAMS.xDirection == com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.EncoderDirection.FORWARD ? "FORWARD":"REVERSE");

        init(hardwareMap);
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
    public Pose2d getTargetArtifactClusterPose(){
        Coords coordsFromLimelight = limelightArtifactDetector.getTargetPosition();
        Coords coordsFromIntake = new Coords(coordsFromLimelight.getX(), coordsFromLimelight.getY(),
                coordsFromLimelight.getZ(), coordsFromLimelight.getAngle());

        return coordsToPose(coordsFromIntake);
    }

    public List<Pose2d> getTopThreeTargetPositions(){
//        List<Coords> coordsFromLimelight = limelightArtifactDetector.getTopThreeTargetPositions();
//
//        List<Pose> result = new ArrayList<>();
//
//        for (Coords location: coordsFromLimelight){
//            Coords coordsFromIntake = new Coords(location.getX(), location.getY(),
//                    location.getZ(), location.getAngle());
//        }
//
//
//        return result;
        return new ArrayList<Pose2d>();
    }

    private void setDriveParams() {

        MecanumDrive.Params p = MecanumDrive.PARAMS;
        p.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        p.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        p.inPerTick = 0.1;
        p.lateralInPerTick = 0.82300130164; // p.inPerTick;
        p.trackWidthTicks = 11.086271752519513;

        // feedforward parameters (in tick units)
        p.kS = 1.4185622916188247;
        p.kV = 0.1311175736849693;
        p.kA = 0.00005;

        // path profile parameters (in inches)
        p.maxWheelVel = 50;
        p.minProfileAccel = -30;
        p.maxProfileAccel = 30;

        // turn profile parameters (in radians)
        p.maxAngVel = Math.PI; // shared with path
        p.maxAngAccel = Math.PI;

        // path controller gains
        p.axialGain = 6.0;
        p.lateralGain = 7.0;
        p.headingGain = 15.0; // shared with turn

        p.axialVelGain = 0.0;
        p.lateralVelGain = 0.0;
        p.headingVelGain = 0.0; // shared with turn


        PinpointDrive.Params pinpointParams = PinpointDrive.PARAMS;
        pinpointParams.xOffset = -6; // pinpoint 5.534895114657507
        pinpointParams.yOffset = 3.5; // tuner y-position: 3.665864020873486, y-intercept: 0

        pinpointParams.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        pinpointParams.xDirection = com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.EncoderDirection.FORWARD;
        pinpointParams.yDirection = com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public List<Coords> getArtifactList(){
        return limelightArtifactDetector.getArtifactCoords();
    }
}
