package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.pedropathing.MecanumDrive;

@TeleOp(name = "Test TeleOp", group = "Test")
public class TeleopTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive robot = (MecanumDrive) RobotFactory.createRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("running",1);
            telemetry.update();
        }
    }
}
