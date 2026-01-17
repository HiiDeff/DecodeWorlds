package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.BlockerTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.ParkTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.task.UnboundedIntakeTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@Config
public abstract class Robot3TeleOp extends LinearOpMode {
    public static int FLYWHEEL_TARGET_RPM = 2600, MANUAL_OVERRIDE_FLYWHEEL_RPM = 2600;
    public static double PIVOT_TARGET_POS = 0.17, MANUAL_OVERRIDE_PIVOT_POS = 0.17;
    public static double FLYWHEEL_ON_INTAKE_POWER = 1.0, FLYWHEEL_WAIT_INTAKE_POWER = -0.2, INTAKE_IDLE_POWER = 0.0;

    public static int RAPID_FIRE_THRESHOLD = 100;
    private TeleOpState state;
    private boolean parking = false;
    private double drivePow = 0.0;
    private Task task;
    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;
    private GamePad gp1, gp2;
    private RobotBase robot;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        sensorUpdateThread = new SensorUpdateThread(robot);

        waitForStart();

        robot.startTeleopDrive();
        robot.startLimelight();
        robot.setLimelightAllianceColor(isRed());
        sensorUpdateThread.start();
        state = TeleOpState.OVERRIDE;

        while (opModeIsActive()) {
            update();
            drive();
            runIntake();
            if(state == TeleOpState.DRIVING) {
                shoot();
            } else if(state == TeleOpState.OVERRIDE) {
                manualOverride();
            }
            park();
        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        gp1.update();
        gp2.update();

        if(task != null && task.perform()) task = null;

        if(gp1.back()&&gp1.onceB()) {
            robot.setPose(new Pose(0, 0, Math.PI));
        }

        robot.updateLimelight();
        robot.updateRobotPoseUsingLimelight();

        if(state == TeleOpState.OVERRIDE) {
            if(gp1.back() && gp1.onceY()) {
                state = TeleOpState.DRIVING;
                if(task != null){
                    task.cancel();
                    task = null;
                }
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                robot.setRampPosition(RampTask.Position.DOWN);
                robot.setFlywheelTargetVelocity(0); FLYWHEEL_TARGET_RPM = 0;
                robot.startTeleopDrive();
            }
        } else {
            if(gp1.back() && gp1.onceY()) {
                state = TeleOpState.OVERRIDE;
                if(task != null){
                    task.cancel();
                    task = null;
                }
            }
        }

        Pose limelightPose = robot.getLimelightRobotPose();
        multipleTelemetry.addData("robot position using limelight", limelightPose.getX()+" "+limelightPose.getY()+" "+limelightPose.getHeading());
        multipleTelemetry.addData("dist to goal", robot.getVectorToGoal().getMagnitude());
        multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
        multipleTelemetry.addData("target rpm", FLYWHEEL_TARGET_RPM);
        multipleTelemetry.update();
    }

    private void manualOverride() {
        robot.setFlywheelTargetVelocity(MANUAL_OVERRIDE_FLYWHEEL_RPM);
        robot.setPivotPosition(MANUAL_OVERRIDE_PIVOT_POS);
        robot.setTurretTargetPosition(0);
        if(!gp1.back() && gp1.onceA() && robot.flywheelAtTarget()) {
            if(task!=null) task.cancel();
            task = Presets.createRapidShootTask(robot);
        }
    }

    private void runIntake(){
        if(gp1.rightTrigger()>0.3) {
            robot.runIntake();
        }
        else if(gp1.leftTrigger()>0.3) {
            robot.runIntakeReversed();
        }
        else if(task == null) {
            if(robot.hasArtifact()) {
                robot.runIntakeWithPower(INTAKE_IDLE_POWER);
            } else {
                robot.stopIntake();
            }
        }
    }

    private void shoot(){
        FLYWHEEL_TARGET_RPM = robot.calcFlywheelRpm();
        PIVOT_TARGET_POS = robot.calcPivotPosition();
        if(robot.artifactState.getBallCount()>=1||task!=null){
            robot.setFlywheelTargetVelocity(FLYWHEEL_TARGET_RPM);
        }else{
            robot.setFlywheelTargetVelocity(0);
        }
        robot.setPivotPosition(PIVOT_TARGET_POS);
        robot.turretAutoAim();

        if(!gp1.back() && gp1.onceA() && (robot.flywheelAtTarget()||!robot.hasArtifact())) {
            if(task!=null) task.cancel();
            if(robot.getVectorToGoal().getMagnitude()>RAPID_FIRE_THRESHOLD) {
                task = Presets.createSlowShootTask(robot);
            } else {
                task = Presets.createRapidShootTask(robot);
            }
        }
    }

    private void park(){
        if(gp1.back()&&gp1.onceA()) {
            parking = !parking;
            robot.setParkPosition(parking ? ParkTask.Position.DOWN : ParkTask.Position.UP);
        }
    }

    private void drive() {
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
        drivePow = Math.max(Math.sqrt(x * x + y * y), Math.abs(a));

        if(state==TeleOpState.DRIVING || state==TeleOpState.OVERRIDE) {
            robot.setTeleOpDrive(x, y, a, true);
        }
    }

    protected abstract boolean isRed();
}
