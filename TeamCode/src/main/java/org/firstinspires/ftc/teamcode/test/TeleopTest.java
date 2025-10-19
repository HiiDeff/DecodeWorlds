package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.util.GamePad;
@Config
@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {

    /*
    VALS
    far shot: 3300 RPM, 28 Tps, 0.63 pivot pos
    20 inch shot: 2500 RPM, 28 tps, 0.53 pivot pos
    adj goal shot: 2500 RPM, 28 tps, 0.48 pivot pos
     */

    public static int FLYWHEEL_TICKS_PER_REVOLUTION = 28;
    public static int FLYWHEEL_RPM = 2500, FLYWHEEL_DELTA = 1;
    public static double INTAKE_POWER = 0.8;

    public static double PIVOT_POS = 0.48, PIVOT_DELTA = 0.0001;
    public static double KICK_DOWN_POS = 0.55;
    public static double KICK_UP_POS = 0.30;
    public static boolean kickerUp, flywheelActive;

    public MultipleTelemetry multipleTelemetry;

    public GamePad gp1, gp2;

    public DcMotorEx flywheelLeft, flywheelRight;
    public DcMotor intakeMotor;

    public Servo kicker;
    public Servo pivotL, pivotR;
    public Servo pusher;
    private Follower robot;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = RobotFactory.createRobot(hardwareMap);
        robot.setStartingPose(new Pose());

        gp1 = new GamePad(gamepad2);
        gp2 = new GamePad(gamepad1);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker = hardwareMap.get(Servo.class, "kicker");
        pivotL = hardwareMap.get(Servo.class, "pivotLeft");
        pivotR = hardwareMap.get(Servo.class, "pivotRight");
        pusher = hardwareMap.get(Servo.class, "pusher");

        pivotL.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        robot.startTeleopDrive();

        while (opModeIsActive()) {

            if(gp2.leftStickY()>0.3){
                PIVOT_POS+=PIVOT_DELTA;
                PIVOT_POS = Math.min(PIVOT_POS, 1.0);
            } else if(gp2.leftStickY()<-0.3) {
                PIVOT_POS-=PIVOT_DELTA;
                PIVOT_POS = Math.max(PIVOT_POS, 0.0);
            }
            pivotL.setPosition(PIVOT_POS);
            pivotR.setPosition(PIVOT_POS);

            if(-gp2.rightStickY()>0.3){
                FLYWHEEL_RPM+=FLYWHEEL_DELTA;
                FLYWHEEL_RPM = Math.min(FLYWHEEL_RPM, 6000);
            } else if(-gp2.rightStickY()<-0.3) {
                FLYWHEEL_RPM-=FLYWHEEL_DELTA;
                FLYWHEEL_RPM = Math.max(FLYWHEEL_RPM, 0);
            }

            if(gp1.onceX()) flywheelActive = !flywheelActive;
            if(flywheelActive) {
                flywheelLeft.setVelocity((FLYWHEEL_RPM/60.0)*FLYWHEEL_TICKS_PER_REVOLUTION);
                flywheelRight.setVelocity((FLYWHEEL_RPM/60.0)*FLYWHEEL_TICKS_PER_REVOLUTION);
            } else {
                flywheelLeft.setPower(0);
                flywheelRight.setPower(0);
            }

            if(gp1.rightTrigger()>0.3) {
                intakeMotor.setPower(INTAKE_POWER);
            } else if(gp1.leftTrigger()>0.3) {
                intakeMotor.setPower(-INTAKE_POWER);
            } else intakeMotor.setPower(0);

            if(gp1.rightBumper()) {
                pusher.setPosition(1.0);
            } else pusher.setPosition(0.5);

            if(gp1.onceA()) {
                kickerUp = !kickerUp;
            }
            kicker.setPosition(kickerUp? KICK_UP_POS:KICK_DOWN_POS);

            robot.update();
            drive();

            gp1.update();
            gp2.update();

            multipleTelemetry.addData("flywheel power", FLYWHEEL_RPM);
            multipleTelemetry.addData("flywheel pos", flywheelRight.getCurrentPosition());
            multipleTelemetry.addData("flywheel rpm", flywheelRight.getVelocity()/28*60);
            multipleTelemetry.update();
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
//        double limPow = driveLim.calculate(pow);
//        if (pow > MIN_DRIVE_POW) {
//            x *= limPow / pow;
//            y *= limPow / pow;
//        }
//        a = turnLim.calculate(a);
//        telemetry.addData("driving", x + " " + y + " " + a);

        robot.setTeleOpDrive(x, y, a, true);
    }
}