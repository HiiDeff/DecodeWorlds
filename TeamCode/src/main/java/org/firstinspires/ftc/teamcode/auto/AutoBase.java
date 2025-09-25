package org.firstinspires.ftc.teamcode.auto;//package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.AutoStates;
import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.task.Task;

@Config
public abstract class AutoBase extends LinearOpMode {

    public static int AA_TOTAL_TIME_MILLIS = 30000;

    public static int AA_NUM_OF_SPIKE_MARKS = 3, AA_NUM_OF_CYCLES = 4;

    public static int INIT_DELAY_TIME = 0;
    public static int AA_TIME_FOR_A_CYCLE = 2000;
    public static int AA_TIME_FOR_PARK = 0000; //0 for now because prioritize delivering a sample over 3 point hang

    // Moving velocity constraints
    public static double HIGH_SPEED = 35;
    public static double LOW_SPEED = 30;

    protected final ElapsedTime timer = new ElapsedTime();
    protected RobotBase robot;

    protected AutoState state = AutoState.START;
    protected AutoStates autoStates = new AutoStates();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = createRobot(hardwareMap);
        resetRobot();
        while (opModeInInit()) {
            telemetry.addData("initializing", 1);
            telemetry.update();
        }

        state = AutoState.START;
        Task task = createStartTask();
        timer.reset();

        ElapsedTime cycleTimer = new ElapsedTime();

        boolean done = false;
        while (opModeIsActive() && !done && task != null) {
            robot.updateEverything();
            if(task.perform()) {
                task = createCycleTask();
            }
            telemetry.addData("status", "running");
            telemetry.addData("position", robot.getPose().getX()+" "+robot.getPose().getY());
            telemetry.addData("heading", robot.getHeading());
            telemetry.update();

            Log.i("edbug loop times", cycleTimer.milliseconds()+"");
            Log.i("el_debug heading_error",robot.getHeadingError()+"");
            cycleTimer.reset();
        }
    }

    private boolean timeToFinish() {
        return timer.milliseconds() > AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_PARK;
    }

    private boolean hasSpikeMarksLeft() {
        return autoStates.getSpikeMarkNumber() < AA_NUM_OF_SPIKE_MARKS;
    }

    private boolean hasTimeForOneMoreCycle() {
        // Go ahead if there are still 10 seconds left and
        return autoStates.getCycleNumber() < AA_NUM_OF_CYCLES &&
                timer.milliseconds() < AA_TOTAL_TIME_MILLIS - AA_TIME_FOR_A_CYCLE;
    }

    private void resetRobot() {
        robot.setStartingPose(getStartingPose());
    }

    protected abstract RobotBase createRobot(HardwareMap hardwareMap);

    protected abstract Task createStartTask();
    protected abstract Task createCycleTask();
    protected abstract Task createFinishTask();
    protected abstract Pose getStartingPose();
    protected abstract boolean isRed();
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
