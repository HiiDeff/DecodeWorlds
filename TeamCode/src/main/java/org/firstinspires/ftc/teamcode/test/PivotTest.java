package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Pivot Test", group="Test")
public class PivotTest extends LinearOpMode {

    public static double leftPivotAmount = 0.5;
    public static double rightPivotAmount = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        Servo rightPivot = hardwareMap.get(Servo.class, "rightPivot");


        waitForStart();
        while (opModeIsActive()) {
            leftPivot.setPosition(leftPivotAmount);
            rightPivot.setPosition(rightPivotAmount);
        }
    }
}
