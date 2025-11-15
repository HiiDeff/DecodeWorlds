package org.firstinspires.ftc.teamcode.test;

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
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
@Config
@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {

    /*
    setup:
    corner of tape measure = on the corner tile under the goal, 10 inches behind the atag
    other end of tape measure = on the edge of the far wall, exactly 2 tiles away from the opposite alliance edge
    goal edge @ ~9.5 inches (sqrt(10^2-3^2))

    dist (inch) ||  RPM   ||  angle (shooter)
   <------------------------------>
        140     ||  4150  ||     0.42
        130     ||  3900  ||     0.4
        120     ||  3800  ||     0.35
        110     ||  3650  ||     0.345
        100     ||  3550  ||     0.33
         90     ||  3400  ||     0.32
         80     ||  3350  ||     0.31
               <----------->
         80     ||  3350  ||     0.27
         70     ||  3200  ||     0.25
         60     ||  3100  ||     0.23
               <----------->
         60     ||  3100  ||     0.13
         50     ||  2900  ||     0.115
         45     ||  2700  ||     0.48 CONTROL POINT/ NOT DONE
         40     ||  2800  ||     0.10
         30     ||  2700  ||     0.085
         20     ||  2600  ||     0.07
         10     ||  N/A  ||     N/A
     */

    public static int FLYWHEEL_RPM = 2500;

    // Pivot DOWN: 0.57
    // Pivot FULL EXTENSION: 1
    public static double PIVOT_POS = 0.57, SERVO_SKIP_CORRECTION = 0.01, INTAKE_POWER = 0.7;
    public static boolean kickerUp, flywheelActive, aiming, autoaim = true;
    private Task task;

    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;

    public GamePad gp1, gp2;
    private RobotBase robot;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.init(new Pose());

        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        waitForStart();

        robot.startTeleopDrive();
        robot.startLimelight();
        robot.setLimelightAllianceColor(false);
        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        while (opModeIsActive()) {
            update();

            multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
            multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());

            if(autoaim) {
                robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
                FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
            } else {
                robot.setPivotPosition(PIVOT_POS);
            }

            if(gp1.onceX()) flywheelActive = !flywheelActive;
            if(flywheelActive) {
                robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
            } else {
                robot.setFlywheelTargetVelocity(0);
            }

            if(gp1.rightTrigger()>0.3) {
                robot.runIntakeWithPower(INTAKE_POWER);
            } else if(gp1.leftTrigger()>0.3) {
                robot.runIntakeReversed();
            } else robot.stopIntake();

//            if(gp1.rightBumper()) {
//                robot.runPusher();
//            } else if(gp1.leftBumper()) {
//                robot.runPusherReversed();
//            } else robot.stopPusher();

            if(gp1.onceA()) {
                kickerUp = !kickerUp;
            }

            if (gp1.onceB()){

            }
            if(kickerUp) {
                robot.setKickerPower(KickerTask.Direction.UP);
                robot.setBlockerPosition(BlockerTask.Position.OPEN);
            } else {
                robot.stopKicker();
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
            }

            drive();

            gp1.update();
            gp2.update();

            multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
            multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
            multipleTelemetry.update();
        }
        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        if(task != null && task.perform()) task = null;
    }

    private void drive() {
        if(gp1.onceY()) {
            aiming = !aiming;
            if(!aiming) {
                robot.startTeleopDrive();
            } else {
                robot.holdPoint(new BezierPoint(robot.getPose()), robot.getAngleToGoal(), false);
            }
        }
        if(aiming) {
        } else {
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
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);

//        robot.setDrivePowers(x, y, a);
            if(Math.abs(a)<0.05) robot.updateRobotPoseUsingLimelight();
            robot.setTeleOpDrive(x, y, a, true);
        }
    }

    private double calcPivotPosition(double x) {
        double coef[] = {
                -0.883007,
                0.1253661,
                -0.00632362,
                0.0001586931,
                -0.000002128523,
                1.561264 * Math.pow(10.0, -8.0),
                -5.90707 * Math.pow(10.0, -11.0),
                9.014571 * Math.pow(10.0, -14.0)

        };

        double pos = 0;
        for(double i =0; i<=7; i++){
            pos+=Math.pow(x, i) *coef[(int)i];
        }

        return pos;
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        double coef[] = {
                4164.83516,
                -330.89418,
                26.92545,
                -1.134829,
                0.02809456,
                -0.0004254504,
                0.000003974666,
                -2.22832 * Math.pow(10.0, -8.0),
                6.85307 * Math.pow(10.0, -11.0),
                -8.8577 * Math.pow(10.0, -14.0)};

        double rpm = 0;
        for(double i = 0; i<=9.0; i++){
            rpm+=Math.pow(distToGoalInches, i) * coef[(int)i];
        }

        return (int)rpm;
    }
}