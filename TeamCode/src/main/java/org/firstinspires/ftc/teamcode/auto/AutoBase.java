package org.firstinspires.ftc.teamcode.auto;//package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.sortauto.close.CloseSortAuto;
import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBase extends LinearOpMode {

    public static int AA_TOTAL_TIME_MILLIS = 30000;
    public static int AA_TIME_FOR_A_CYCLE = 4000;
    public static int AA_TIME_FOR_PARK = 1500;

    public static Location firstLocation = Location.MID;

    protected final ElapsedTime timer = new ElapsedTime();
    protected RobotBase robot;
    protected AutoState state;
    protected Task globalTask;
    protected AutoStates autoStates = new AutoStates();
    protected SensorUpdateThread sensorUpdateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap);
        resetRobot();
        sensorUpdateThread = new SensorUpdateThread(robot);
        state = AutoState.START;
        globalTask = createStartTask();
        firstLocation = getFirstLocation();

        telemetry.addData("FIRST INTAKE LOCATION (far auton)", firstLocation);
        telemetry.addData("CYCLE COUNT", getNumOfCycles());
        telemetry.update();

        waitForStart();

        sensorUpdateThread.start();
        timer.reset();
        ElapsedTime cycleTimer = new ElapsedTime();

        boolean done = false;
        while (opModeIsActive() && !done && globalTask != null) {
            robot.updateEverything();
            robot.updateLimelight();
            if(timeToFinish() && state!=AutoState.FINISH && !(this instanceof CloseSortAuto)) {
                globalTask.cancel();
                globalTask = createFinishTask();
            } else if(globalTask.perform()) {
                if(hasTimeForOneMoreCycle()){
                    autoStates.setCycleNumber(autoStates.getCycleNumber()+1);
                    globalTask = createCycleTask();
                } else if(state != AutoState.FINISH) {
                    globalTask = createFinishTask();
                }
            }
            telemetry.addData("position", robot.getPose().getX()+", "+robot.getPose().getY());
            telemetry.addData("heading", robot.getHeading());
            telemetry.update();

            Log.i("edbug loop times", cycleTimer.milliseconds()+"");
            cycleTimer.reset();
        }
        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private boolean hasTimeForOneMoreCycle() {
        return autoStates.getCycleNumber() < getNumOfCycles() &&
                timer.milliseconds() < AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_A_CYCLE;
    }

    private void resetRobot() {
        robot.autoInit();
        robot.startLimelight();
        //TODO: this works, but not all 3 lines are necessary
        robot.setStartingPose(getStartingPose());
        robot.setPose(getStartingPose());
        robot.updateHeading(getStartingPose().getHeading());
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);
    protected abstract int getNumOfCycles();
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
