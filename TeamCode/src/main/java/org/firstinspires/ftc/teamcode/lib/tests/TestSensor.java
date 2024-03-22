package org.firstinspires.ftc.teamcode.lib.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TestSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor sensor = hardwareMap.get(TouchSensor.class, "touch sensor");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("sensor", sensor.getValue());
            telemetry.update();
        }
    }
}
