package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.drive.robot1.Robot1;
import org.firstinspires.ftc.teamcode.task.ArtifactReadyCondition;
import org.firstinspires.ftc.teamcode.task.ConditionalParallelTask;
import org.firstinspires.ftc.teamcode.task.ConditionalTask;
import org.firstinspires.ftc.teamcode.task.DecisionTask;
import org.firstinspires.ftc.teamcode.task.FlywheelTask;
import org.firstinspires.ftc.teamcode.task.IntakeTask;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.ParallelTask;
import org.firstinspires.ftc.teamcode.task.SeriesTask;
import org.firstinspires.ftc.teamcode.task.SleepTask;
import org.firstinspires.ftc.teamcode.task.PusherTask;
import org.firstinspires.ftc.teamcode.util.GamePad;

@TeleOp(name = "V1 TeleOp")
@Config
public class V1RobotTeleOp extends LinearOpMode {

    public static int FLYWHEEL_SHOOT_TIME, FLYWHEEL_WARM_UP_TIME;

    public static double FLYWHEEL_VELOCITY, PUSHER_POWER;


    public MultipleTelemetry multipleTelemetry;
    public ElapsedTime elapsedTime = new ElapsedTime();

    private Robot1 robot;

    private GamePad gp1, gp2;

    private SensorUpdateThread sensorUpdateThread;

    SeriesTask shootTask;

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

    private void runIntake(){
        // Operator controlled
        if (gp2.rightTrigger() > 0.1){
            robot.runIntake();
        } else if (gp2.leftTrigger() > 0.1) {
            robot.runIntakeReversed();
        } else {
            robot.stopIntake();
        }
    }

    private SeriesTask createShooterTask(){
        return new SeriesTask(
                new FlywheelTask(robot, FLYWHEEL_VELOCITY, FLYWHEEL_WARM_UP_TIME + FLYWHEEL_SHOOT_TIME),
                new SleepTask(FLYWHEEL_WARM_UP_TIME),
                new KickerTask(robot, KickerTask.Position.UP),
                new SleepTask(FLYWHEEL_SHOOT_TIME),
                new KickerTask(robot, KickerTask.Position.DOWN)
        );
    }

    private void runShooter(){
        // Operator Controlled

        if (gp1.rightBumper()){
            if (shootTask != null){
                shootTask.cancel();
            }
            shootTask = new SeriesTask(
                    new DecisionTask(
                            () -> robot.hasArtifact(),
                            createShooterTask(),
                            new SeriesTask(
                                    new ConditionalParallelTask(
                                            () -> !robot.hasArtifact(),
                                            new IntakeTask(robot,robot.INTAKE_POWER, 3000),
                                            new PusherTask(robot, false, 3000)

                                    ),
                                    new ConditionalTask(() -> robot.hasArtifact()),
                                    createShooterTask()
                            )
                    )
            );
        }


        // TODO: Manual override of pivot position

        // TODO: Toggle to automatically turn towards goal
    }

    private void update(){
        if (shootTask != null && shootTask.perform()){shootTask = null;}
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
