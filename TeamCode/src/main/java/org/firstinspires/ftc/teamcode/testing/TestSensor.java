package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

@TeleOp(name = "test color sensor", group = "test")
public class TestSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor sensor = hardwareMap.get(ColorSensor.class, "colorv2");

        waitForStart();
        while(opModeIsActive()){
            sleep(30);
            telemetry.addData("red", sensor.red());
            telemetry.addData("blue", sensor.blue());
            telemetry.addData("green", sensor.green());
            telemetry.update();
        }
    }
}
