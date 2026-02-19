package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.filter.MovingAverage;

/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
public class PinpointDrive extends MecanumDrive {
    public static class Params {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        //These are tuned for 3110-0002-0001 Product Insight #1
        // RR localizer note: These units are inches, presets are converted from mm (which is why they are inexact)
        public double xOffset = 3.25; // 3.1091754774386136 according to RR
        public double yOffset = 1.625; //1.9439441107387374 according to RR

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.

        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    public static Params PARAMS = new Params();
    public GoBildaPinpointDriverRR pinpoint;
    private Pose2d lastPinpointPose = pose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(pose);
        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");

        // RR localizer note: don't love this conversion (change driver?)
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));

        pinpoint.setEncoderResolution(PARAMS.encoderResolution);

        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override otos pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            pinpoint.setPosition(pose);
        }
        pinpoint.update();
        pose = pinpoint.getPositionRR();
        lastPinpointPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        updateAcceleration();

        return pinpoint.getVelocityRR();
    }

    // Moving Average Filters
    private ElapsedTime timer = null;
    private final MovingAverage angularVelocityFilter = new MovingAverage(3);
    private final MovingAverage angularAccelerationFilter = new MovingAverage(4);
    private double lastHeading = 0.0, lastFilteredAngularVelocity = 0.0;
    private final MovingAverage accelXFilter = new MovingAverage(3);
    private final MovingAverage accelYFilter = new MovingAverage(3);
    public static double heading = 0;

    private void updateAcceleration() {

        if (timer == null) {
            timer = new ElapsedTime();
            lastHeading = heading;
            lastFilteredAngularVelocity = 0.0;
            return;
        }
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 1e-6 || dt > 1) return;

        // Angular Velocity

        double deltaHeading = Utils.normalize(heading - lastHeading);
        lastHeading = heading;

        angularVelocityFilter.add(deltaHeading / dt);
        double filteredAngularVelocity = angularVelocityFilter.get();

        double angularAcceleration = (filteredAngularVelocity - lastFilteredAngularVelocity) / dt;
        lastFilteredAngularVelocity = filteredAngularVelocity;

        angularAccelerationFilter.add(angularAcceleration);

        // Translational Velocity

        // TODO: IMPLEMENT acceleration and velocity
        //accelXFilter.add(pinpoint.getVelocityRR().linearVel.x.get(1));
        //accelYFilter.add(getAcceleration().getYComponent());


        //Log.i("pedro translational velo", "("+pinpoint.getVelocityRR().getXComponent()+" "+this.getVelocity().getYComponent()+")");
        //Log.i("pedro translational accel", "("+pinpoint.getVelocityRR().getXComponent()+" "+this.getAcceleration().getYComponent()+")");
        Log.i("edbug translational accel x", accelXFilter.get()+"");
        Log.i("edbug translational accel y", accelYFilter.get()+"");

        Log.i("edbug angular velo", angularVelocityFilter.get()+""); //pedro angular velo doesn't work
        Log.i("edbug angular accel", angularAccelerationFilter.get()+"");

        // Heading for transfer between teleop and auto
    }

    public void updateHeading() {
        heading = pinpoint.getHeading(AngleUnit.RADIANS);
        Log.i("edbug raw heading", heading+"");
    }

    public double getHeading() {
        return heading;
    }


    public Pose2d getPose(){
        return pose;
    }

    public double getTranslationalVelocity(){
        // TODO: imeplement;
        return 0.0;
    }

    public Pose2d getPositionRR() {
        return pinpoint.getPositionRR();
    }


    // for debug logging
    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }



}