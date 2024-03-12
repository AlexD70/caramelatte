package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class TestMotor extends LinearOpMode {

    static public double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "A");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            motor.setPower(speed);
        }
    }
}
