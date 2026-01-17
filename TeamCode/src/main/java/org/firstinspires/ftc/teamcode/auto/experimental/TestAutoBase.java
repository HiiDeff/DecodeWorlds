package org.firstinspires.ftc.teamcode.auto.experimental;//package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Location;
import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class TestAutoBase extends LinearOpMode {

    public static Location firstLocation = Location.MID;
    public static int AA_TOTAL_TIME_MILLIS = 30000;

    public static int AA_NUM_OF_CYCLES = 3;

    public static int INIT_DELAY_TIME = 0;
    public static int AA_TIME_FOR_A_CYCLE = 2000;
    public static int AA_TIME_FOR_PARK = 1500;

    protected final ElapsedTime timer = new ElapsedTime();
    protected RobotBase robot;

    protected AutoState state = AutoState.START;
    protected AutoStates autoStates = new AutoStates();
    protected SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap);
        resetRobot();

        state = AutoState.START;
        Task task = createStartTask();
        firstLocation = getFirstLocation();
        telemetry.addData("FIRST INTAKE LOCATION (only for far auton)",firstLocation);
        telemetry.update();

        waitForStart();

        sensorUpdateThread = new SensorUpdateThread(robot);
        sensorUpdateThread.start();

        timer.reset();
        ElapsedTime cycleTimer = new ElapsedTime();
        boolean done = false;
        while (opModeIsActive() && !done && task != null) {
            robot.updateEverything();
            if(task.perform()) {
                if(hasTimeForOneMoreCycle()){
                    autoStates.setCycleNumber(autoStates.getCycleNumber()+1);
                    task = createCycleTask();
                } else {
                    task = createFinishTask();
                }
            }
            telemetry.addData("position", robot.getPose().getX()+", "+robot.getPose().getY());
            telemetry.addData("heading", robot.getHeading());
            telemetry.update();

            Log.i("edbug loop times", cycleTimer.milliseconds()+"");
            cycleTimer.reset();
        }
        sensorUpdateThread.interrupt();
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private boolean hasTimeForOneMoreCycle() {
        return autoStates.getCycleNumber() < AA_NUM_OF_CYCLES &&
                timer.milliseconds() < AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_A_CYCLE;
    }

    private void resetRobot() {
        robot.autoInit();
        robot.setStartingPose(getStartingPose());
        robot.setPose(getStartingPose());
        robot.updateHeading(Math.PI); //cheese
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);
    protected abstract Location getFirstLocation();
    protected abstract Task createStartTask();
    protected abstract Task createCycleTask();
    protected abstract Task createFinishTask();
    protected abstract Pose getStartingPose();
    protected int getSign() {
        return isRed() ? -1 : 1;
    }
    protected abstract boolean isRed();
    protected abstract boolean isFar();
    protected double toR(double degree) {
        return Math.toRadians(degree);
    }

    // Reverse the direction and convert the degree to radius.
    protected double toOppositeR(double degree) {
        return Math.toRadians(degree + 180);
    }

    protected enum AutoState {
        START,
        CYCLE,
        FINISH
    }
}
