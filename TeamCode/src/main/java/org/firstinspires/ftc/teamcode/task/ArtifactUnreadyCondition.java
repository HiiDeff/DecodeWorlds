package org.firstinspires.ftc.teamcode.task;

import org.firstinspires.ftc.teamcode.common.Condition;
import org.firstinspires.ftc.teamcode.drive.RobotBase;

public class ArtifactUnreadyCondition implements Condition {

    private final RobotBase robot;

    public ArtifactUnreadyCondition(RobotBase robot){
        this.robot = robot;
    }

    public boolean shouldContinue(){
        return !robot.hasArtifact();
    }
}
