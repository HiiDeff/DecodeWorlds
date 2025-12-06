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
import org.firstinspires.ftc.teamcode.task.RampTask;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
import org.firstinspires.ftc.teamcode.util.Utils;

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
        130     ||  4050  ||     0.48
        120     ||  3900  ||     0.48 DONE
        110     ||  3700  ||     0.36 DONE
        100     ||  3600  ||     0.34 DONE
         90     ||  3450  ||     0.32 DONE
         80     ||  3250  ||     0.32 DONE
               <----------->
         80     ||  3250  ||     0.32 DONE
         70     ||  3150  ||     0.31 DONE
         60     ||  3000  ||     0.3  DONE
               <----------->
         60     ||  3000  ||     0.3  DONE
         50     ||  2900  ||     0.29 DONE
         40     ||  2800  ||     0.27 DONE
         30     ||  2700  ||     0.21 DONE
         20     ||  2600  ||     0.14 DONE
         10     ||  N/A  ||     N/A
     */

    public static int FLYWHEEL_RPM = 2500;

    // Pivot DOWN: 0.57
    // Pivot FULL EXTENSION: 1
    public static double PIVOT_POS = 0.4, SERVO_SKIP_CORRECTION = 0.01, INTAKE_POWER = 0.7;
    public static double TURRET_TICKS_PER_RADIANS = 103.8*2.0, TURRET_TARGET_RAD = 0.0;
    public static boolean isRed = false;
    public static boolean rampUp, flywheelActive, aiming, autoaim = false, turretActive = true;
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
        robot.setLimelightAllianceColor(isRed);
        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        while (opModeIsActive()) {
            update();
            robot.updateLimelight();

            Pose robotPose = robot.turret.calcRobotPose(robot.limelightAprilTagDetector.getLimelightFieldPose());
            multipleTelemetry.addData("robot position using limelight", robotPose.getX()+" "+robotPose.getY());

            if(turretActive) {
                robot.setTurretTargetPosition(TURRET_TARGET_RAD);
            } else {
                robot.setTurretTargetPosition(0);
            }

            robot.setPivotPosition(PIVOT_POS);


//            if(autoaim) {
//                robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
//                FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
//            } else {
//                robot.setPivotPosition(PIVOT_POS);
//            }
            robot.setPivotPosition(PIVOT_POS);
//
            if(gp1.onceX()) flywheelActive = !flywheelActive;
            if(flywheelActive) {
                robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
            } else {
                robot.setFlywheelTargetVelocity(0);
            }
//
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
                rampUp = !rampUp;
            }
//
//            if (gp1.onceB()){
//
//            }
            if(rampUp) {
                robot.setRampPosition(RampTask.Position.UP);
                robot.setBlockerPosition(BlockerTask.Position.OPEN);
            } else {
                robot.setRampPosition(RampTask.Position.DOWN);
                robot.setBlockerPosition(BlockerTask.Position.CLOSE);
            }

            drive();

            gp1.update();
            gp2.update();

//            multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
//            multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
            multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
            multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
            multipleTelemetry.addData("turret pos", robot.getTurretAngleTicks());
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
                8.55474359,
                -1.46832254,
                0.106241899,
                -0.00422199996,
                0.000102803617,
                -0.00000160149163,
                1.60354007 * Math.pow(10.0, -8.0),
                -9.98609081 * Math.pow(10.0, -11.0),
                3.51949442 * Math.pow(10.0, -13.0),
                -5.36055472 * Math.pow(10.0, -16.0)

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
                -2753.23427,
                834.76971,
                -53.89067,
                1.88809,
                -0.0389984,
                0.00048851,
                -0.00000363913,
                1.48046 * Math.pow(10.0, -8.0),
                -2.52911 * Math.pow(10.0, -11.0)};

        double rpm = 0;
        distToGoalInches = Utils.clamp(distToGoalInches, 20, 130);
        for(double i = 0; i<=8.0; i++){
            rpm+=Math.pow(distToGoalInches, i) * coef[(int)i];
        }

        return (int)rpm;
    }
}