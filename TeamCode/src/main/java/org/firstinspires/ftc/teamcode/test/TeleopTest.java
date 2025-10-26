package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.PivotTask;
import org.firstinspires.ftc.teamcode.util.GamePad;
@Config
@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {

    /*
    setup:
    corner of tape measure = on the corner tile under the goal, 7 inches from each edge
    other end of tape measure = on the edge of the far wall, exactly 2 tiles away from the opposite alliance edge
    goal edge @ 7 inches
    VALS:
    2500 RPM: (0 -> 38 inch) z(x) = (3.29783*10^{-7})x^{4}+0.0000260577x^{3}-0.00063992x^{2}+0.00162601x+0.582999
    - 7 inch = 0.57
    - 10 inch = 0.56
    - 15 inch = 0.53
    - 20 inch = 0.51
    - 25 inch = 0.50
    - 30 inch = 0.50
    - 35 inch = 0.47
    - 40 inch = 0.45
    3000 RPM: (38 -> X inch)
    -
    3300 RPM: (X -> 151 inch)
    -

    trying solution 2: scale RPM linearly based on dist, from 7=2500RPM to 150=3500RPM (0.30)
    then interpolate angles
    7 inch = 2500 RPM, 150 inch = 3500 RPM
    rpm = 1000/143(dist-7)+2500
    dist (inch) ||  RPM   ||  angle (shooter)
   <------------------------------>
        130     ||  3360  ||     0.320
        120     ||  3290  ||     0.320
        110     ||  3220  ||     0.310
        100     ||  3150  ||     0.310
         90     ||  3080  ||     0.310
         80     ||  3010  ||     0.300
         70     ||  2941  ||     0.310
         60     ||  2871  ||     0.330
         50     ||  2801  ||     0.330
         45     ||  2766  ||     0.330
         40     ||  2731  ||     0.490
         30     ||  2661  ||     0.515
         20     ||  2591  ||     0.550
         10     ||  2521  ||     0.560
          7     ||  2500  ||     0.570

    for distances less than 45: y = -0.0000295544x^{2}-0.00102978x+0.577059
    for distances at least 45: y = -1.00701*10^{-8}x^{4}+0.00000350981x^{3}-0.000428815x^{2}+0.0213042x-0.0381544

    ideas:
    gear down the second gear on the intake
    TODO spin pusher backwards before shooting
     */

    public static int FLYWHEEL_RPM = 2500;
    public static boolean kickerUp, flywheelActive, aiming;

    public MultipleTelemetry multipleTelemetry;

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

        while (opModeIsActive()) {
            robot.updateEverything();
            robot.updateRobotPoseLimelight();

            multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
            multipleTelemetry.addData("robot pos", robot.getPose().getHeading());

            robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
            FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());

            if(gp1.onceX()) flywheelActive = !flywheelActive;
            if(flywheelActive) {
                robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
            } else {
                robot.setFlywheelTargetVelocity(0);
            }

            if(gp1.rightTrigger()>0.3) {
                robot.runIntake();
            } else if(gp1.leftTrigger()>0.3) {
                robot.runIntakeReversed();
            } else robot.stopIntake();

            if(gp1.rightBumper()) {
                robot.runPusher();
            } else if(gp1.leftBumper()) {
                robot.runPusherReversed();
            } else robot.stopPusher();

            if(gp1.onceA()) {
                kickerUp = !kickerUp;
            }
            robot.setKickerPosition(kickerUp? KickerTask.Position.UP:KickerTask.Position.DOWN);

            drive();

            gp1.update();
            gp2.update();

            multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
            multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
            multipleTelemetry.update();
        }
    }

    private void drive() {
        if(gp1.onceY()) {
            aiming = !aiming;
            if(!aiming) {
                robot.startTeleopDrive();
            } else {
                robot.followPath(
                        robot.pathBuilder()
                                .addPath(new BezierLine(robot.getPose(), robot.getPose()))
                                .setLinearHeadingInterpolation(robot.getHeading(), 0)
                                .build()
                );
            }
        }
        if(!aiming) {
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
            robot.setTeleOpDrive(x, y, a, true);
        }
    }

    private double calcPivotPosition(double x) {
        if(x<45) {
           return (-0.0000295544)*Math.pow(x,2)-0.00102978*x+0.577059;
        } else {
            return (-1.00701*Math.pow(10, -8))*Math.pow(x,4)+0.00000350981*Math.pow(x, 3)-0.000428815*Math.pow(x, 2)+0.0213042*x-0.0381544;
        }
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        return (int)(1000.0/143*(distToGoalInches-7)+2500);
    }
}