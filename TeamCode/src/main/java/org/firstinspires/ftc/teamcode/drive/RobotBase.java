package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;

public abstract class RobotBase extends MecanumDrive {

    // Constants
    public static double INTAKE_POWER = 0.5, OUTTAKE_POWER = 0.5;
    public static double PUSHER_POWER = 1.0;

    // Common
    protected final HardwareMap hardwareMap;

    // Motors
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    private boolean flywheelOn;
    public final FlywheelPID flywheelPID;
    public final DcMotorEx intake;
    private boolean intakeOn;

    // Servos
    public final Servo leftPivot;
    public final Servo rightPivot;
    public final Servo kicker;
    public final CRServo pusher;

    // Sensors
    public final RevColorSensorV3 leftColorSensor;
    public final RevColorSensorV3 rightColorSensor;

    // States
    ArtifactState artifactState;

    // Cached Encoder Values
    private double flywheelVelocityTicksPerSecond = 0.0;

    public RobotBase(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(hardwareMap, followerConstants, driveConstants, localizer, pathConstraints);
        this.hardwareMap = hardwareMap;
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Motors:
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheelOn = false;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeOn = false;
        // Servos:
        kicker = hardwareMap.get(Servo.class, "kicker");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        pusher = hardwareMap.get(CRServoImplEx.class, "pusher");
        // Sensors:
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "rightColorSensor");

        // Motion Control:
        flywheelPID = new FlywheelPID(this, getVelocityPIDCoefficients());
    }

    ///////////////////* INIT *///////////////////
    public void teleOpInit() {}

    public void autoInit() {}
    ///////////////////* UPDATES *///////////////////
    public void updateEverything() {
        updatePoseEstimate();
        updateEncoders();
        updateSensors();
        updateProfilers();
        updatePIDs();
    }

    private void updatePoseEstimate() {
        update();
    }

    private void updateEncoders() {
        flywheelVelocityTicksPerSecond = leftFlywheel.getVelocity();
    }

    public void updateSensors() { //public for multithreading
        artifactState.update();
    }

    private void updateProfilers() {}

    private void updatePIDs() {
        if(flywheelOn) setFlywheelPower(flywheelPID.getPower());
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

    ///////////////////* PUSHER UTILS *///////////////////
    public void runPusher() {
        pusher.setPower(PUSHER_POWER);
    }

    public void runPusherReversed() {
        pusher.setPower(-PUSHER_POWER);
    }

    public void stopPusher() {
        pusher.setPower(0);
    }

    ///////////////////* KICKER UTILS *///////////////////
    public abstract double getKickerPosition(KickerTask.Position position);

    public void setKickerPosition(KickerTask.Position position){
        kicker.setPosition(getKickerPosition(position));
    }

    ///////////////////* FLYWHEEL UTILS *///////////////////
    public abstract PIDCoefficients getVelocityPIDCoefficients();

    public void setFlywheelPower(double power){
        leftFlywheel.setPower(Utils.clamp(power, -1, 1));
        rightFlywheel.setPower(Utils.clamp(power, -1, 1));
    }

    public void setFlywheelTargetVelocity(double velocityTicksPerSecond){
        flywheelPID.setTargetVelocity(velocityTicksPerSecond);
        flywheelOn = true;
    }

    public double getFlywheelVelocity(){
        return flywheelVelocityTicksPerSecond;
    }

    public void stopFlywheel(){
        setFlywheelTargetVelocity(0.0);
        flywheelOn = false;
    }

    ///////////////////* PIVOT UTILS *///////////////////
    public abstract double getPivotTargetPos(PivotTask.WhichPivot pivot, PivotTask.Position position);

    public void setPivotPosition(PivotTask.Position position){
        leftPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.LEFT, position));
        rightPivot.setPosition(getPivotTargetPos(PivotTask.WhichPivot.RIGHT, position));
    }

    ///////////////////* COLOR SENSOR UTILS *///////////////////

    public boolean hasArtifact(){
        return artifactState.getArtifactState();
    }

}