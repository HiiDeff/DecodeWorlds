package org.firstinspires.ftc.teamcode.task;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.Condition;
import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class ArtifactReadyCondition implements Condition {

    private final RobotBase robot;

    public ArtifactReadyCondition(RobotBase robot){
        this.robot = robot;
    }

    public boolean shouldContinue(){
        if(robot.hasArtifact()) Log.i("edbug", "artifact detected!");
        return robot.hasArtifact();
    }
}
