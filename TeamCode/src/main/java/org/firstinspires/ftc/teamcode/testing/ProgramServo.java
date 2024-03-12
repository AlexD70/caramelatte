package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ProgramServo extends LinearOpMode {
    public static double servoL = 0d, servoR = 0d;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoLeft = hardwareMap.get(Servo.class, "leftarm");
        Servo servoRight = hardwareMap.get(Servo.class, "rightarm");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            servoLeft.setPosition(servoL);
            servoRight.setPosition(servoR);
        }
    }
}
