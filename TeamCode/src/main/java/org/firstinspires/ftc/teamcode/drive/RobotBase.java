package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.limelight.AprilTagType;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightAprilTagDetector;
import org.firstinspires.ftc.teamcode.util.limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.pid.VelocityPIDCoefficients;

@Config
public abstract class RobotBase extends MecanumDrive {

    // Constants
    public static double INTAKE_POWER = 1, OUTTAKE_POWER = -0.8;
    public static double PUSHER_POWER = 1.0;
    public static double TURRET_TICKS_PER_RAD = 384.5*2.0;

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
    public static PIDCoefficients TURRET_PID_COEFFICIENTS = new PIDCoefficients(0.0, 1.0, 0.01, 0.0, 0.4);
    public final DcMotorEx intake;
    private boolean intakeOn;

    // Servos
    public final Servo leftPivot;
    public final Servo rightPivot;
    public final Servo blocker;
    public final Servo ramp;

    // Sensors
    public final RevColorSensorV3 leftColorSensor;
    public final RevColorSensorV3 rightColorSensor;

    // Camera
    public final Limelight3A limelight;
    public final LimelightAprilTagDetector limelightAprilTagDetector;
    public static LimelightConfig LLConfig = new LimelightConfig(640, 480,
            0, 54,41,
            -3,0,0);
    public static int APRIL_TAG_PIPELINE = 1;

    // States
    public final ArtifactState artifactState;

    // Cached Encoder Values
    private double flywheelVelocityTicksPerSecond = 0.0;
    private int turretAngleTicks = 0;

    public RobotBase(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(hardwareMap, followerConstants, driveConstants, localizer, pathConstraints);
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Motors:
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        rightPivot.setDirection(Servo.Direction.REVERSE);
        blocker = hardwareMap.get(Servo.class, "blocker");
        ramp = hardwareMap.get(Servo.class, "ramp");
        // Sensors:
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "rightColorSensor");
        // Limelight:
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightAprilTagDetector = new LimelightAprilTagDetector(limelight, LLConfig);
        // Motion Control:
        flywheelPID = new FlywheelPID(this, getVelocityPIDCoefficients());
        turret = new Turret(this, TURRET_PID_COEFFICIENTS, TURRET_TICKS_PER_RAD);
        artifactState = new ArtifactState(this);
    }

    ///////////////////* INIT *///////////////////
    public void teleOpInit() {
        setPivotPosition(PivotTask.Position.MID);
        setBlockerPosition(BlockerTask.Position.CLOSE);
    }

    public void autoInit() {
        setPivotPosition(PivotTask.Position.MID);
        setBlockerPosition(BlockerTask.Position.CLOSE);
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
    public void setTurretPower(double power){
        turretMotor.setPower(Utils.clamp(power, -1, 1));
    }
    public void setTurretTargetPosition(double angleRad){
        turret.setTargetAngle((int)(angleRad*TURRET_TICKS_PER_RAD));
    }
    public int getTurretAngleTicks() {
        return turretAngleTicks;
    }

    ///////////////////* PIVOT UTILS *///////////////////
    public abstract double getPivotTargetPos(PivotTask.WhichPivot pivot, PivotTask.Position position);

    public void setPivotPosition(double position) {
        leftPivot.setPosition(position);
        rightPivot.setPosition(position);
    }
    public void setPivotPosition(PivotTask.Position position){
        leftPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.LEFT, position));
        rightPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.RIGHT, position));
    }

    ///////////////////* COLOR SENSOR UTILS *///////////////////

    public boolean hasArtifact(){
        return artifactState.getArtifactState();
    }

    ///////////////////* LIMELIGHT UTILS *///////////////////
    public void startLimelight() {
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);
        limelight.start();
    }
    public void setLimelightAllianceColor(boolean isRedAlliance) {
        limelightAprilTagDetector.setAllianceColor(isRedAlliance);
    }
    public void updateLimelight() { // public for teleop
        limelightAprilTagDetector.updateLimelight();
    }
    public void stopLimelight() {
        limelight.stop();
    }
    public void updateRobotPoseUsingLimelight() {
        Pose limelightFieldPose = limelightAprilTagDetector.getLimelightFieldPose();
        if(limelightFieldPose != null) {
            Pose robotPose = turret.calcRobotPose(limelightFieldPose);
            this.setPose(robotPose);
        }
    }
    public double getDistToGoalInches() {
        double dist = this.getPose().distanceFrom(new Pose()) + LLConfig.xOffset;
        Log.i("edbug dist to goal", ""+dist);
        return dist;
    }
    public double getAngleToGoal() {
        return Math.atan2(-this.getPose().getY(), -this.getPose().getX());
    }
    public AprilTagType getMotif() {
        return limelightAprilTagDetector.getMotif();
    }
}