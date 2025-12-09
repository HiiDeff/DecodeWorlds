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
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;

@Config
@TeleOp(name="Robot 2 TeleOp", group="TeleOp")
public abstract class Robot3TeleOp extends LinearOpMode {
    public static int FLYWHEEL_TARGET_RPM = 2600, MANUAL_OVERRIDE_FLYWHEEL_RPM = 2600;
    public static double PIVOT_TARGET_POS = 0.07, MANUAL_OVERRIDE_PIVOT_POS = 0.07;
    public static double FLYWHEEL_ON_INTAKE_POWER = 1, INTAKE_IDLE_POWER = 0.3;
    public static TeleOpState state;
    public static double drivePow = 0.0;
    private Task task;
    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;
    public GamePad gp1, gp2;
    private RobotBase robot;

    private boolean shooting = false;

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
        state = TeleOpState.DRIVING;

        while (opModeIsActive()) {
            update();
            drive();
            if(state == TeleOpState.DRIVING) {
                runIntake();
            } else if(state == TeleOpState.OVERRIDE) {
                runIntake();
                manualOverride();
            }
            shoot();

        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        gp1.update();
        gp2.update();

        if(task != null && task.perform()) task = null;

        if(gp1.onceY()) {
            robot.holdPoint(new BezierPoint(robot.getPose()), robot.getVectorToGoal().getTheta(), false);
            state = TeleOpState.AIMING;
        }

        if (state == TeleOpState.AIMING){
            if(drivePow>0.25) {
                state = TeleOpState.DRIVING;
                if(task != null){
                    task.cancel();
                    task = null;
                }
                robot.updateRobotPoseUsingLimelight(); // only update robot pose here
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                robot.setRampPosition(RampTask.Position.DOWN);
                robot.setFlywheelTargetVelocity(0); FLYWHEEL_TARGET_RPM = 0;
                robot.startTeleopDrive();
            }
        }

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
        }else{
            robot.updateLimelight();
            robot.updateRobotPoseUsingLimelight();

            if(gp1.back() && gp1.onceY()) {
                state = TeleOpState.OVERRIDE;
                if(task != null){
                    task.cancel();
                    task = null;
                }
            }
        }

        multipleTelemetry.addData("dist to goal", robot.getVectorToGoal().getMagnitude());
        multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
        multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
        multipleTelemetry.addData("target rpm", FLYWHEEL_TARGET_RPM);
        multipleTelemetry.update();
    }

    private void manualOverride() {
        robot.setFlywheelTargetVelocity(MANUAL_OVERRIDE_FLYWHEEL_RPM);
        robot.setPivotPosition(MANUAL_OVERRIDE_PIVOT_POS);
        if(gp1.onceA() && robot.flywheelAtTarget()) {
            task = new ParallelTask(
                    new BlockerTask(robot, BlockerTask.Position.OPEN),
                    new IntakeTask(robot, FLYWHEEL_ON_INTAKE_POWER, false, 150000),
                    new RampTask(robot, RampTask.Position.DOWN)
            );
        } else if(drivePow>0.25) {
            if(task != null) {
                task.cancel();
                task = null;
            }
            robot.setBlockerPosition(BlockerTask.Position.CLOSE);
            robot.setRampPosition(RampTask.Position.DOWN);
        }
    }

    private void runIntake(){
        if (!shooting){
            if(gp1.rightTrigger()>0.3) {
                robot.runIntake();
            }
            else if(gp1.leftTrigger()>0.3) {
                robot.runIntakeReversed();
            }
            else if(task==null) {
                robot.runIntakeWithPower(INTAKE_IDLE_POWER);
            }
        }
    }

    private void shoot(){
        autoAim();
        if(gp1.onceA()) {
            shooting = !shooting;
        }

        if (shooting){
            robot.setRampPosition(RampTask.Position.UP);
            robot.setBlockerPosition(BlockerTask.Position.OPEN);
            robot.runIntake();
        }else{
            robot.setRampPosition(RampTask.Position.DOWN);
            robot.setBlockerPosition(BlockerTask.Position.CLOSE);
        }
    }

    private void autoAim(){
        FLYWHEEL_TARGET_RPM = robot.calcFlywheelRpm();
        PIVOT_TARGET_POS = robot.calcPivotPosition();
        robot.setFlywheelTargetVelocity(FLYWHEEL_TARGET_RPM);
        robot.setPivotPosition(PIVOT_TARGET_POS);
        robot.turretAutoAim();
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
