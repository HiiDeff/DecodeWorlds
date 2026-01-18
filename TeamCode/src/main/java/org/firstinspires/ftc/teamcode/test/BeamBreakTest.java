package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Disabled
@Config
@TeleOp(name="BeamBreak Test", group="Test")
public class BeamBreakTest extends LinearOpMode {
    public static double POS = 0.0;
    public static double MOTORPOWER = 0;

    public MultipleTelemetry multipleTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");

        DigitalChannel breakbeamoutput = hardwareMap.get(DigitalChannel.class, "breakbeamoutput");
        breakbeamoutput.setMode(DigitalChannel.Mode.OUTPUT);
        DigitalChannel breakbeamreceiver = hardwareMap.get(DigitalChannel.class, "breakbeamreceiver");
        breakbeamreceiver.setMode(DigitalChannel.Mode.INPUT);

        breakbeamoutput.setState(true);

        boolean prevState = true;
        boolean currState = true;
        int count = 0;
        long prevTime = 0;
        long currTime = 0;
        boolean empty = true;
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(MOTORPOWER);
            currState = breakbeamreceiver.getState();
            currTime = System.currentTimeMillis();
            if (!currState)
            {
                empty = false;
            }
            if (currState != prevState)
            {
                prevTime = currTime;
            }
            else if (currState && (currTime - prevTime > 100)){
                empty = true;
            }

            if (!currState && prevState)
            {
                count++;
            }
            multipleTelemetry.addData("receiver", currState);
            multipleTelemetry.addData("empty", empty);
            multipleTelemetry.addData("count", count);
            multipleTelemetry.addData("output", breakbeamoutput.getState());
            multipleTelemetry.update();
            prevState = currState;
        }
    }
}