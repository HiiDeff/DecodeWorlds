package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
public abstract class Robot1TeleOp extends LinearOpMode {
    // Far: 4000 rpm, 1.0 pivot
    public static int FLYWHEEL_RPM = 2600, MANUAL_OVERRIDE_FLYWHEEL_RPM = 2600;
    public static double MANUAL_OVERRIDE_PIVOT_POS = 0.07;
    public static double FLYWHEEL_ON_INTAKE_POWER = 0.7;
    public static boolean kickerUp, aiming, intaking = true, blocking = true, autoaim = true, shooting = false; // when autoaim is false use manual override
    private double RPMs[] = {2600, 2600, 2600, 2700, 2800, 2900, 3100, 3200, 3350, 3400, 3550, 3650, 3800, 3900, 4150, 4150};
    private double lowerBoundDist[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};
    private double pivotAngles[] = {0.07, 0.07, 0.07, 0.085, 0.10, 0.115, 0.13, 0.25, 0.27, 0.32, 0.33, 0.345, 0.35, 0.4, 0.42, 0.42};

    private Task task;
    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;
    public GamePad gp1, gp2;
    private RobotBase robot;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        robot.setStartingPose(new Pose());
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        sensorUpdateThread = new SensorUpdateThread(robot);

        waitForStart();

        robot.startTeleopDrive();
        robot.startLimelight();
        robot.setLimelightAllianceColor(isRed());
        sensorUpdateThread.start();

        robot.runIntake();

        while (opModeIsActive()) {
            update();
            updateAiming();
            updateIntake();
            updateShooting();
            updateDrive();
            updateUnjamming();
        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        if(task != null && task.perform()) task = null;

        if(gp1.back() && gp1.onceY()) {
            autoaim = !autoaim;
        }

        gp1.update();
        gp2.update();

        multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
        multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
        multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
        multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
        multipleTelemetry.update();
    }

    private void updateShooting() {
        if(!autoaim||aiming && task == null) {
            Log.i("adebug", "Flywheel done: " + robot.flywheelPID.isDone());
            if(gp1.onceA()) {
                if(shooting || robot.flywheelPID.isDone()) {
                    shooting = !shooting;
                    if (shooting){
                        robot.setBlockerPosition(BlockerTask.Position.OPEN);
                        robot.setKickerPower(KickerTask.Direction.UP);
                    }else{
                        robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                        robot.kicker.setPower(0);
                    }
                }
            }
        } if(!autoaim) {
            if(gp1.onceA()) {
                kickerUp = !kickerUp;
//                robot.setKickerPosition(kickerUp? KickerTask.Position.UP:KickerTask.Position.DOWN);
            }
        }
    }

    private void updateIntake() {

        if (!aiming){
            robot.setBlockerPosition(BlockerTask.Position.CLOSE);
        }

        // Override
        if (gp2.rightBumper()){
            if (intaking){
                robot.stopIntake();
            } else if (!intaking){
                if(robot.getFlywheelState()){
                    robot.runIntakeWithPower(FLYWHEEL_ON_INTAKE_POWER);
                } else {
                    robot.runIntake();
                }
            }
            intaking = !intaking;
        }
        if (gp2.onceB()){
            robot.setBlockerPosition(blocking ? BlockerTask.Position.OPEN : BlockerTask.Position.CLOSE);
            blocking = !blocking;
        }
    }

    private void updateAiming() {
        if(autoaim) {
            if(gp1.onceX()) {
                aiming = !aiming;
                if(!aiming) {
                    robot.startTeleopDrive();
                    robot.setFlywheelTargetVelocity(0);
                    robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                    robot.kicker.setPower(0);
                } else {
                    robot.holdPoint(new BezierPoint(robot.getPose()), robot.getAngleToGoal(), false);
                    robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
                }
            }
            if(task==null) robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
            FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
        } else {
            robot.setPivotPosition(MANUAL_OVERRIDE_PIVOT_POS);
            robot.setFlywheelTargetVelocity(MANUAL_OVERRIDE_FLYWHEEL_RPM);
        }
    }

    private void updateDrive() {
        double x = 0, y = 0, a = 0;
        if (gp1.dpadLeft() || gp1.dpadRight()) {
            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
        } else if (gp1.dpadUp() || gp1.dpadDown()) {
            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
        } else {
            x = -gp1.leftStickY();
            y = -gp1.leftStickX();
            a = -gp1.rightStickX() * 0.5;
        }
        double pow = Math.sqrt(x * x + y * y);

        if(pow>0.5) {
            if(aiming) {
                aiming = false;
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                robot.kicker.setPower(0);
                if(task != null) {
                    task.cancel();
                    task = null;
                }
//                robot.setKickerPosition(KickerTask.Position.DOWN); kickerUp = false;
                robot.setFlywheelTargetVelocity(0);
                robot.startTeleopDrive();
            }
        }

        if(Math.abs(a)<0.05 && !aiming) robot.updateRobotPoseUsingLimelight();
        if(!aiming) robot.setTeleOpDrive(x, y, a, true);
    }
    private void updateUnjamming() {
        if(gp2.back() && gp2.onceLeftBumper()) {
            if(aiming) {
                aiming = false;
                robot.setBlockerPosition(BlockerTask.Position.OPEN);
                robot.setKickerPower(KickerTask.Direction.DOWN);
                if(task != null) {
                    task.cancel();
                    task = null;
                }
//                robot.setKickerPosition(KickerTask.Position.DOWN); kickerUp = false;
                robot.setFlywheelTargetVelocity(0);
                robot.startTeleopDrive();
            }
            task = Presets.createUnjammingTask(robot);
        }
    }

    private double calcPivotPosition(double distToGoalInches) {
        distToGoalInches = Utils.clamp(distToGoalInches, 0, 149);
        int idx = (int)Math.floor(distToGoalInches/10.0);
        double mod = distToGoalInches - idx*10;
        Log.i("edbug calcs", idx+" "+mod);
        double pivotAngle = pivotAngles[idx] + (pivotAngles[idx+1]-pivotAngles[idx])/(lowerBoundDist[idx+1]-lowerBoundDist[idx])*mod;
        return pivotAngle;
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        distToGoalInches = Utils.clamp(distToGoalInches, 0, 149);
        int idx = (int)Math.floor(distToGoalInches/10.0);
        double mod = distToGoalInches - idx*10;
        double rpm = RPMs[idx] + (RPMs[idx+1]-RPMs[idx])/(lowerBoundDist[idx+1]-lowerBoundDist[idx])*mod;
        return (int)rpm;
    }

    protected abstract boolean isRed();
}