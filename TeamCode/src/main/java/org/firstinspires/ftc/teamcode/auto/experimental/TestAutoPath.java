package org.firstinspires.ftc.teamcode.auto.experimental;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;

@Autonomous(name = "Test Auto", group = "Test")
public class TestAutoPath extends TestAuto {

    @Override
    protected RobotBase createRobot(HardwareMap hardwareMap) {
        return (RobotBase) RobotFactory.createRobot(hardwareMap);
    }

    @Override
    protected Pose getStartingPose() {
        return new Pose(0,0,Math.toRadians(0));
    }

    @Override
    protected boolean isRed() {
        return false;
    }
    @Override
    protected BezierCurve getBezierCurve1() {
        return null;
    }

    @Override
    protected BezierCurve getBezierCurve2() {
        return null;
    }

    @Override
    protected BezierCurve getBezierCurve3() {
        return null;
    }

    @Override
    protected BezierCurve getBezierCurve4() {
        return null;
    }
}
