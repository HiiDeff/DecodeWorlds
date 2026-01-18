package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Disabled
@TeleOp(name = "PID Test", group = "Test")
@Config

public class PIDTest extends LinearOpMode {

    public MultipleTelemetry multipleTelemetry;
    private RobotBase robot;
    public static int targetRPM = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {

            robot.setFlywheelTargetVelocity(targetRPM);

            robot.updateEverything();
            multipleTelemetry.addData("current position", robot.getFlywheelVelocityRpm());
            multipleTelemetry.addData("PID target position", targetRPM);
            multipleTelemetry.update();

        }

    }
}
