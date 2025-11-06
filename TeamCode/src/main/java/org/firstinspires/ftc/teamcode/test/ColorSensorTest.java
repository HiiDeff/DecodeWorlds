package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.objectdetector.ImageProcessor;

@TeleOp(name ="ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {

    ColorRangeSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(ColorRangeSensor.class, "color");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("hue: ", ImageProcessor.ColorToHsv(sensor.getNormalizedColors()).h);
            telemetry.addData("sat: ", ImageProcessor.ColorToHsv(sensor.getNormalizedColors()).s);
            telemetry.addData("val: ", ImageProcessor.ColorToHsv(sensor.getNormalizedColors()).v);
            telemetry.addData("distance: ", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
