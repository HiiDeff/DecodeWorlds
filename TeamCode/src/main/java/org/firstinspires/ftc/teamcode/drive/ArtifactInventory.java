package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class ArtifactInventory {
    public int numOfArtifacts = 0;
    int CURRENT_3_BALL_THRESHOLD = 2200;
    int CURRENT_2_BALL_THRESHOLD = 1200;
    int CURRENT_1_BALL_THRESHOLD = 500;

    private RobotBase robot;

    public ArtifactInventory(RobotBase robot){
        this.robot = robot;
    }

    public void updateArtifactCount(){
        double current = robot.intake.getCurrent(CurrentUnit.MILLIAMPS);
        Log.i("ndbug intake current", "" +current);

        if(current > CURRENT_3_BALL_THRESHOLD) numOfArtifacts=3;
        else if(current > CURRENT_2_BALL_THRESHOLD) numOfArtifacts=2;
        else if(current > CURRENT_1_BALL_THRESHOLD) numOfArtifacts =1;
        Log.i("ndbug number artifacts", ""+numOfArtifacts);
    }

}
