package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.ParkTask;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.AprilTagType;
import org.firstinspires.ftc.teamcode.util.limelight.Coords;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightAprilTagDetector;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightArtifactDetector;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;

@Config
public abstract class RobotBase extends MecanumDrive {

    // Constants
    public static double INTAKE_POWER = 1, OUTTAKE_POWER = -0.8;
    public static double PUSHER_POWER = 1.0;
    public static double TURRET_TICKS_PER_RAD = 384.5*2.0/Math.PI;

    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final FlywheelPID flywheelPID;
    private boolean flywheelOn;
    public final DcMotorEx turretMotor;
    public final Turret turret;
    private boolean turretOn;
    public static PIDCoefficients TURRET_PID_COEFFICIENTS = new PIDCoefficients(0.0, 1.0, 0.006, 0.0, 0.0, 0.0001);
    public final DcMotorEx intake;
    private boolean intakeOn;

    // Servos
    public final Servo leftPivot;
    public final Servo rightPivot;
    public final Servo blocker;
    public final Servo ramp;
    public final Servo leftPark;
    public final Servo rightPark;

    // Sensors
    public final RevColorSensorV3 backColor1;
    public final RevColorSensorV3 backColor2;
    public final RevColorSensorV3 frontColor1;
    public final RevColorSensorV3 frontColor2;

    // Camera
    public final Limelight3A limelight;
    public final LimelightAprilTagDetector limelightAprilTagDetector;
    public final LimelightArtifactDetector limelightArtifactDetector;
    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            Math.toRadians(10), 54.371,42.318,
            0,0,12.5);
    public static int ARTIFACT_PIPELINE = 0;
    public static int APRIL_TAG_PIPELINE = 2;

    // States
    public final ArtifactState artifactState;
    public Pose limelightRobotPose; // estimated robot pose if limelight sees ATag, otherwise null
    public Pose limelightTransOffset = new Pose(0, 0); //offset values based on last seen ATag values
    private boolean detectingAprilTags = true;

    // Cached Encoder Values
    private double flywheelVelocityTicksPerSecond = 0.0;
    private int turretAngleTicks = 0;

    // Lights
    private final RevBlinkinLedDriver light;

    public RobotBase(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, PinpointLocalizer localizer, PathConstraints pathConstraints) {
        super(hardwareMap, followerConstants, driveConstants, localizer, pathConstraints);
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Motors:
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheelOn = false;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeOn = false;
        // Servos:
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        leftPivot.setDirection(Servo.Direction.REVERSE);
        rightPivot.setDirection(Servo.Direction.FORWARD);
        blocker = hardwareMap.get(Servo.class, "blocker");
        ramp = hardwareMap.get(Servo.class, "ramp");
        leftPark = hardwareMap.get(Servo.class, "leftPark");
        rightPark = hardwareMap.get(Servo.class, "rightPark");
        // Sensors:
        frontColor1 = hardwareMap.get(RevColorSensorV3.class, "frontColor1");
        frontColor2 = hardwareMap.get(RevColorSensorV3.class, "frontColor2");
        backColor1 = hardwareMap.get(RevColorSensorV3.class, "backColor1");
        backColor2 = hardwareMap.get(RevColorSensorV3.class, "backColor2");
        // Limelight:
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightAprilTagDetector = new LimelightAprilTagDetector(limelight, LLConfig);
        limelightArtifactDetector = new LimelightArtifactDetector(limelight, LLConfig);
        // Motion Control:
        flywheelPID = new FlywheelPID(this, getVelocityPIDCoefficients());
        turret = new Turret(this, TURRET_PID_COEFFICIENTS, TURRET_TICKS_PER_RAD);
        artifactState = new ArtifactState(this);
        // Lights:
        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");
    }

    ///////////////////* INIT *///////////////////
    public void teleOpInit() {
        setMaxPower(1.0);
        setPivotPosition(PivotTask.Position.MID);
        setBlockerPosition(BlockerTask.Position.CLOSE);
        setParkPosition(ParkTask.Position.UP);
        setStartingPose(new Pose(100, 0, heading));
    }

    public void autoInit() {
        resetTurret();
        heading = 0;
        setMaxPower(1.0);
        setTurretTargetPosition(0.0);
        setPivotPosition(PivotTask.Position.MID);
        setBlockerPosition(BlockerTask.Position.CLOSE);
        setParkPosition(ParkTask.Position.UP);
        setRampPosition(RampTask.Position.DOWN);
    }
    ///////////////////* UPDATES *///////////////////
    public void updateEverything() {
        updatePoseEstimate();
        updateEncoders();
        updateProfilers();
        updatePIDs();
//        updateLimelight();
//        updateSensors(); //handled by thread
    }

    private void updatePoseEstimate() {
        update();
    }

    private void updateEncoders() {
        flywheelVelocityTicksPerSecond = leftFlywheel.getVelocity();
        turretAngleTicks = turretMotor.getCurrentPosition();
    }

    public void updateSensors() { //public for multithreading
        artifactState.update();
    }

    private void updateProfilers() {}

    private void updatePIDs() {
        setFlywheelPower(flywheelPID.getPower());
        setTurretPower(turret.getPower());
    }

    ///////////////////* INTAKE UTILS *///////////////////
    public void runIntakeWithPower(double power) {
        intake.setPower(power);
        intakeOn = true;
    }

    public void runIntake() {
        intake.setPower(INTAKE_POWER);
        intakeOn = true;
    }

    public void runIntakeReversed() {
        intake.setPower(OUTTAKE_POWER);
        intakeOn = true;
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeOn = false;
    }
    public void toggleIntake() {
        intakeOn = !intakeOn;
        if(intakeOn) runIntake();
        else stopIntake();
    }

    ///////////////////* BLOCKER UTILS *///////////////////
    public abstract double getBlockerPosition(BlockerTask.Position position);

    public void setBlockerPosition(BlockerTask.Position position){
        blocker.setPosition(getBlockerPosition(position));
    }



    ///////////////////* KICKER UTILS *///////////////////
    public abstract double getRampPosition(RampTask.Position position);

    public void setRampPosition(RampTask.Position position){
        ramp.setPosition(getRampPosition(position));
    }

    ///////////////////* FLYWHEEL UTILS *///////////////////
    public abstract VelocityPIDCoefficients getVelocityPIDCoefficients();

    public void setFlywheelPower(double power){
        if(!flywheelOn || flywheelPID.getTarget()==0) power = 0; //cut power
        leftFlywheel.setPower(Utils.clamp(power, -1, 1));
        rightFlywheel.setPower(Utils.clamp(power, -1, 1));
    }

    public void setFlywheelTargetVelocity(double velocityRpm){
        flywheelPID.setTargetVelocity(velocityRpm*28/60);
        flywheelOn = (velocityRpm != 0);
    }

    public boolean getFlywheelState(){
        return flywheelOn;
    }

    public double getFlywheelVelocityRpm(){
        return flywheelVelocityTicksPerSecond/28*60;
    }

    public double getFlywheelVelocityTicksPerSecond() {
        return flywheelVelocityTicksPerSecond;
    }

    public boolean flywheelAtTarget(){
        return flywheelPID.isDone();
    }

    public abstract double calcPivotPosition();
    public abstract int calcFlywheelRpm();

    ///////////////////* TURRET UTILS *///////////////////
    public void resetTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setTurretPower(double power){
        turretMotor.setPower(Utils.clamp(power, -1, 1));
    }
    public void setTurretTargetPosition(double angleRad){
        angleRad = Utils.normalize(angleRad);
        angleRad = Utils.clamp(angleRad, -Math.PI/2, Math.PI/2);
        turret.setTargetAngle((int)(angleRad*TURRET_TICKS_PER_RAD));
    }
    public int getTurretAngleTicks() {
        return turretAngleTicks;
    }
    public void turretAutoAim() {
        double angleToGoal = getVectorToGoal().getTheta();
        setTurretTargetPosition(Utils.normalize(angleToGoal-getHeading()));
    }

    public boolean turretAtTarget(){
        return turret.isDone();
    }

    ///////////////////* PIVOT UTILS *///////////////////
    public abstract double getPivotTargetPos(PivotTask.WhichPivot park, PivotTask.Position position);

    public void setPivotPosition(double position) {
        leftPivot.setPosition(position);
        rightPivot.setPosition(position);
    }
    public void setPivotPosition(PivotTask.Position position){
        leftPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.LEFT, position));
        rightPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.RIGHT, position));
    }

    ///////////////////* COLOR SENSOR + LIGHTS UTILS *///////////////////

    public boolean hasArtifact(){
        return artifactState.getArtifactState();
    }

    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        light.setPattern(pattern);
    }

    ///////////////////* PARK UTILS *///////////////////
    public abstract double getParkPosition(ParkTask.WhichPark pivot, ParkTask.Position position);

    public void setParkPosition(double position) {
        leftPark.setPosition(position);
        rightPark.setPosition(position);
    }
    public void setParkPosition(ParkTask.Position position){
        leftPark.setPosition(getParkPosition(ParkTask.WhichPark.LEFT, position));
        rightPark.setPosition(getParkPosition(ParkTask.WhichPark.RIGHT, position));
    }

    ///////////////////* LIMELIGHT UTILS *///////////////////
    public void startLimelight() {
        startAprilTagPipeline();
        limelight.start();
    }

    public void startAprilTagPipeline() {
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);
        detectingAprilTags = true;
    }

    public void startArtifactPipeline() {
        limelight.pipelineSwitch(ARTIFACT_PIPELINE);
        detectingAprilTags = false;
    }

    public void setLimelightAllianceColor(boolean isRedAlliance) {
        limelightAprilTagDetector.setAllianceColor(isRedAlliance);
    }
    public void updateLimelight() { // public for teleop usage, auto for motifs
        if (detectingAprilTags){
            limelightAprilTagDetector.update();
        }else{
            limelightArtifactDetector.update();
        }
    }
    public void stopLimelight() {
        limelight.stop();
    }
    public void updateRobotPoseUsingLimelight() {
        Pose limelightFieldPose = limelightAprilTagDetector.getLimelightFieldPose();
        if(limelightFieldPose != null) { // if ATag is detected
            limelightRobotPose = turret.calcRobotPose(limelightFieldPose); //returns null if angular velocity exceeds threshold
        } else {
            limelightRobotPose = null;
        }
        if(limelightRobotPose != null) {
            limelightTransOffset = new Pose(
                limelightRobotPose.getX()-getPose().getX(),
                limelightRobotPose.getY()-getPose().getY()
            );
        }
        limelightAprilTagDetector.updateVectorToGoal(getLimelightRobotPose(), getTranslationalVelocity());
    }
    public Pose getLimelightRobotPose() {
        return (limelightRobotPose==null) ? getPose().copy().plus(limelightTransOffset) : limelightRobotPose;
    }
    public Vector getVectorToGoal() {
        return limelightAprilTagDetector.getVectorToGoal();
    }
    public double getRawDistToGoal() {
        return limelightAprilTagDetector.getRawDistToGoal();
    }
    public AprilTagType getMotif() {
        return limelightAprilTagDetector.getMotif();
    }

    public Coords getTargetArtifactClusterCoords(){return limelightArtifactDetector.getTargetPosition();}
    public abstract Pose getTargetArtifactClusterPose();

    public Pose coordsToPose(Coords coords){
        // x is forward, y is left-right

        Pose robotPose = getPose();
        double robotHeading = robotPose.getHeading();

        // Rotation of coords by robotHeading, then translate by robot position
        // Coords: This means we have to switch x/y, and invert x
        //      +y
        //  -x robot +x
        //      -y
        //
        double x = robotPose.getX() + coords.getY() * Math.cos(robotHeading) - (-coords.getX()) * Math.sin(robotHeading);
        double y = robotPose.getY() + (coords.getY() * Math.sin(robotHeading) - coords.getX() * Math.cos(robotHeading));

        return new Pose(x, y, robotHeading);
    }
}