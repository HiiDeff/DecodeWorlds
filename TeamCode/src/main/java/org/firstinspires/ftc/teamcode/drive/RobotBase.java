package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.pid.PIDCoefficients;

public abstract class RobotBase extends MecanumDrive {

    public static double INTAKE_POWER, OUTTAKE_POWER;


    protected final HardwareMap hardwareMap;

    public final DcMotorEx flywheelLeft, flywheelRight;
    public final DcMotor intake;

    public final Servo kicker;
    public final Servo pivotLeft, pivotRight;
    public final Servo pusher;

    public final FlywheelPID flywheelPID;


    public RobotBase(HardwareMap hardwareMap, FollowerConstants followerConstants, MecanumConstants driveConstants, Localizer localizer, PathConstraints pathConstraints) {
        super(hardwareMap,
                followerConstants,
                driveConstants,
                localizer,
                pathConstraints
        );
        this.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //get hardware

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "motor");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "motor2");
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "motor3");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker = hardwareMap.get(Servo.class, "servo3");
        pivotLeft = hardwareMap.get(Servo.class, "servo");
        pivotRight = hardwareMap.get(Servo.class, "servo2");
        pusher = hardwareMap.get(Servo.class, "servo4");

        flywheelPID = new FlywheelPID(this, getVelocityPIDCoefficients());
    }

    ///////////////////* INIT *///////////////////
    public void teleOpInit(){

    }

    public void autoInit()  {

    }

    ///////////////////* UPDATES *///////////////////
    public void updateEverything(){
        update();
        updatePIDs();
    }

    public void updatePIDs() {
        flywheelPID.updatePID(getVelocityPIDCoefficients());
        setFlywheelPower(getVelocityPIDCoefficients().feedForward + flywheelPID.getPower());
    }

    ///////////////////* INTAKE UTILS *///////////////////
    public void runIntakeWithPower(double power){
        intake.setPower(power);
    }

    public void runIntakeIn() {
        intake.setPower(INTAKE_POWER);
    }

    public void runIntakeOut(){
        intake.setPower(OUTTAKE_POWER);}

    public void stopIntake(){
        intake.setPower(0);
    }

    ///////////////////* KICKER UTILS *///////////////////
    public abstract double getKickerPosition(KickerTask.KickerPosition position);

    public void setKickerPosition(KickerTask.KickerPosition position){
        kicker.setPosition(getKickerPosition(position));
    }

    ///////////////////* PUSHER UTILS *///////////////////
    public void setPusherPosition(double position){
        pusher.setPosition(position);
    }



    ///////////////////* FLYWHEEL UTILS *///////////////////
    public abstract PIDCoefficients getVelocityPIDCoefficients();

    public void setFlywheelPower(double power){
        flywheelLeft.setPower(Utils.clamp(power, -1, 1));
        flywheelRight.setPower(Utils.clamp(power, -1, 1));
    }

    public void setFlywheelVelocity(double velocityTicksPerSecond){
        flywheelPID.setTargetVelocity(velocityTicksPerSecond);
    }

    public double getFlywheelVelocity(){
        return (double)(flywheelLeft.getVelocity());
    }

    public void stopFlywheel(){
        setFlywheelVelocity(0.0);
    }

    ///////////////////* PIVOT UTILS *///////////////////
    public abstract double getPivotPosition(PivotTask.PivotPosition position);

    public void setPivotPosition(PivotTask.PivotPosition position){
        pivotLeft.setPosition(getPivotPosition(position));
        pivotRight.setPosition(getPivotPosition(position));
    }

}