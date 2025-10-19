package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SensorUpdateThread extends Thread {

    private final RobotBase robot;

    public static int THREAD_SLEEP_TIME = 10 ;
    public SensorUpdateThread(RobotBase robot) {
        this.robot = robot;
    }
    private ElapsedTime elapsedTime;
    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
//        elapsedTime = new ElapsedTime();
        try {
            while(true) {
                robot.updateSensors();
                Thread.sleep(THREAD_SLEEP_TIME);
//                Log.i("edbug sensor loop times: ", ""+elapsedTime.milliseconds());
//                elapsedTime.reset();
            }
        } catch (InterruptedException e) {
            Log.e("RCActivity", "SensorUpdateThread interrupted");
        }
    }
}
