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
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;

@Config
@TeleOp(name="NereekshTestTeleop", group="test")
public class Robot1TeleopNereeksh extends LinearOpMode {
    public static int FLYWHEEL_RPM = 2600, MANUAL_OVERRIDE_FLYWHEEL_RPM = 2600;
    public static double PIVOT_POS = 0.07, MANUAL_OVERRIDE_PIVOT_POS = 0.07;
    public static double FLYWHEEL_ON_INTAKE_POWER = 0.7;
    public static boolean isRed = false;
    public static boolean aiming = false;
    private boolean holdingPoint = false;
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
        robot.setLimelightAllianceColor(isRed);
        sensorUpdateThread.start();

        while (opModeIsActive()) {
            update();
            runIntake();
            drive();
            autoAim();
            shoot();
        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        if(task != null && task.perform()) task = null;

        gp1.update();
        gp2.update();

        //state handler/setting states
        if(gp1.onceX()&&!aiming){
            aiming = true;
            robot.holdPoint(new BezierPoint(robot.getPose()), robot.getAngleToGoal(), false);
        }

        multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
        multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
        multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
        multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
        multipleTelemetry.update();
    }

    private void runIntake(){
        //only want to intake when we are not shooting
        if(!aiming){
            if(task != null){
                task.cancel();
                task = null;
            }
            if(gp1.rightTrigger()>0.3){
                robot.runIntake();
            }
            else if (gp1.leftTrigger()>0.3){
                robot.runIntakeReversed();
            }
            else{
                robot.stopIntake();
            }
        }
    }

    private void autoAim(){
        if(aiming){
            FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
            PIVOT_POS = calcPivotPosition(robot.getDistToGoalInches());
            robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
            robot.setPivotPosition(PIVOT_POS);
        }
    }

    private void shoot(){
        if(aiming && robot.flywheelAtTarget()){
            if(gp1.onceA()){//start intaking for shoot when u press A
                task = new SeriesTask(
                        new ParallelTask(
                                new BlockerTask(robot, BlockerTask.Position.OPEN),
                                new IntakeTask(robot, FLYWHEEL_ON_INTAKE_POWER, false, 150000),
                                new KickerTask(robot, KickerTask.Direction.UP, 150000)
                        )
                );
            }
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
        double pow = Math.sqrt(x * x + y * y);

        if(pow>0.5||Math.abs(a)>0.25) {
            if(aiming) {
                aiming = false;
                if(task != null){
                    task.cancel();
                    task = null;
                }
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
                robot.kicker.setPower(0);
                robot.setFlywheelTargetVelocity(0);
                robot.startTeleopDrive();
                robot.updateRobotPoseUsingLimelight();
            }
        }

        if(!aiming) {
            if(pow<0.05 && Math.abs(a)<0.05) {
                robot.updateRobotPoseUsingLimelight();
            }
            robot.setTeleOpDrive(x, y, a, true);
        }
    }

    private double calcPivotPosition(double x) {
        double coef[] = {
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
        x = Utils.clamp(x, 20, 120);

        double pos = 0;
        for(double i =0; i<=9.0; i++){
            pos+=Math.pow(x, i) *coef[(int)i];
        }

        return pos;
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        double coef[] = {
                -6775.27473,
                1568.71241,
                -110.08847,
                4.2639629,
                -0.10028502,
                0.0014919149,
                -0.000014100386,
                8.2002501 * Math.pow(10.0, -8.0),
                -2.674096 * Math.pow(10.0, -10.0),
                3.7399219 * Math.pow(10.0, -13.0)};

        double rpm = 0;
        distToGoalInches = Utils.clamp(distToGoalInches, 20, 140);
        for(double i = 0; i<=9.0; i++){
            rpm+=Math.pow(distToGoalInches, i) * coef[(int)i];
        }

        return (int)rpm;
    }
}
