package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.GamePad;
@Config
@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {

    /*
    VALS
    far shot: 3300 RPM, 28 Tps, 0.63 pivot pos
    20 inch shot: 2500 RPM, 28 tps, 0.53 pivot pos
    adj goal shot: 2500 RPM, 28 tps, 0.5 pivot pos
     */

    public static int FLYWHEEL_TICKS_PER_REVOLUTION;
    public static int FLYWHEEL_RPM = 5538, FLYWHEEL_DELTA = 1;
    public static double INTAKE_POWER = 0.5;

    public static double PIVOT_POS = 0.58, PIVOT_DELTA = 0.000001;
    public static double KICK_DOWN_POS = 0.9;
    public static double KICK_UP_POS = 0.65;
    public static boolean kickerUp, flywheelActive;

    public MultipleTelemetry multipleTelemetry;

    public GamePad gp1;

    public DcMotorEx flywheelLeft, flywheelRight;
    public DcMotor intakeMotor;

    public Servo kicker;
    public Servo pivotL, pivotR;
    public Servo pusher;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        gp1 = new GamePad(gamepad1);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "motor");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "motor2");
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = hardwareMap.get(DcMotor.class, "motor3");

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kicker = hardwareMap.get(Servo.class, "servo3");
        pivotL = hardwareMap.get(Servo.class, "servo");
        pivotR = hardwareMap.get(Servo.class, "servo2");
        pusher = hardwareMap.get(Servo.class, "servo4");

        pivotL.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if(gp1.leftStickY()>0.3){
                PIVOT_POS+=PIVOT_DELTA;
                PIVOT_POS = Math.min(PIVOT_POS, 1.0);
            } else if(gp1.leftStickY()<-0.3) {
                PIVOT_POS-=PIVOT_DELTA;
                PIVOT_POS = Math.max(PIVOT_POS, 0.0);
            }
            pivotL.setPosition(PIVOT_POS);
            pivotR.setPosition(PIVOT_POS);

            if(-gp1.rightStickY()>0.3){
                FLYWHEEL_RPM+=FLYWHEEL_DELTA;
                FLYWHEEL_RPM = Math.min(FLYWHEEL_RPM, 6000);
            } else if(-gp1.rightStickY()<-0.3) {
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
            } else intakeMotor.setPower(0);

            if(gp1.rightBumper()) {
                pusher.setPosition(1.0);
            } else pusher.setPosition(0.5);

            if(gp1.onceA()) {
                kickerUp = !kickerUp;
            }
            kicker.setPosition(kickerUp? KICK_UP_POS:KICK_DOWN_POS);

            gp1.update();

            multipleTelemetry.addData("flywheel power", FLYWHEEL_RPM);
            multipleTelemetry.addData("flywheel pos", flywheelRight.getCurrentPosition());
            multipleTelemetry.update();
        }
    }
}