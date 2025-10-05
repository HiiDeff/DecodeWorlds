package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Motors Test", group="Test")
public class MotorsTest extends LinearOpMode {
    public static double POW = 0.0, POW2 = 0.0, BOTHPOW = 0.0, POW3 = 0.0;
    public static double BOTH_POS = 0.5, SERVO_POS = 0.5, SERVO_POS_2 = 0.5, SERVO_POS_3 = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        motor2.setDirection(DcMotorEx.Direction.REVERSE);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        servo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            if(BOTHPOW!=0) {
                motor1.setPower(BOTHPOW);
                motor2.setPower(BOTHPOW);
            } else {
                motor1.setPower(POW);
                motor2.setPower(POW2);
            }
            if(BOTH_POS != 0.0) {
                servo.setPosition(BOTH_POS);
                servo2.setPosition(BOTH_POS);
            } else {
                servo.setPosition(SERVO_POS);
                servo2.setPosition(SERVO_POS_2);
            }
            motor3.setPower(POW3);
            servo3.setPosition(SERVO_POS_3);
        }
    }
}