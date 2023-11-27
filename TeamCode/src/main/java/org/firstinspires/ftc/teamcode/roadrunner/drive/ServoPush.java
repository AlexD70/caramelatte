package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoPush extends LinearOpMode {

    Servo servo;    Gamepad gamepad1;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servoPush");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.x){
                servo.setPosition(0.4);
            }
        }
    }
}
