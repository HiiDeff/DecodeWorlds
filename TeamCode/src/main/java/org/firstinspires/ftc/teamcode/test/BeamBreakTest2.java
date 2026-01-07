package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@TeleOp(name ="Beam Break Sensor Test 2")
public class BeamBreakTest2 extends LinearOpMode {

    DigitalChannel sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DigitalChannel.class, "beambreakoutput");
        sensor.setMode(DigitalChannel.Mode.OUTPUT);

        waitForStart();
        while(opModeIsActive()){
            if (sensor.getState()) { // Beam is NOT broken (HIGH)
                telemetry.addData("Status", "Beam Present");
            } else {
                telemetry.addData("Status", "Beam Broken!");
            }

            telemetry.update();
        }
    }
}
