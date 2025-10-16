package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "V1 TeleOp")
@Config
public class V1RobotTeleOp extends LinearOpMode {

    public MultipleTelemetry multipleTelemetry;
    public ElapsedTime elapsedTime = new ElapsedTime();

    private Robot1 robot;

    private GamePad gp1, gp2;

    private SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = (Robot1) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();

        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);

        robot.init(new Pose());

        waitForStart();

        robot.startTeleopDrive();

        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        while (opModeIsActive()){
            drive();

            elapsedTime.reset();

            update();
        }
    }

    private void update(){
        gp1.update();
        gp2.update();
        updateTelemetry();
        robot.updateEverything();
    }

    private void updateTelemetry(){
        telemetry.update();
    }

    private void drive() {
        double x = 0, y = 0, a = 0;
//        if (gp1.dpadLeft() || gp1.dpadRight()) {
//            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
//        } else if (gp1.dpadUp() || gp1.dpadDown()) {
//            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
//        } else {
        x = -gp1.leftStickY();
        y = -gp1.leftStickX();
        a = -gp1.rightStickX() * 0.7;
//        }
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
