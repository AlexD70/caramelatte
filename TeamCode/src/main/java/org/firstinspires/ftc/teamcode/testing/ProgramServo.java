package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ProgramServo extends LinearOpMode {
    public static double servoL = 0.3d, servoR = 0.3d, rotat = 0.468d;//pozitie mijloc 0.468d

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoLeft = hardwareMap.get(Servo.class, "leftarm");
        Servo servoRight = hardwareMap.get(Servo.class, "rightarm");
        servoRight.setDirection(Servo.Direction.REVERSE);
        Servo rotate = hardwareMap.get(Servo.class, "rotate");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            servoLeft.setPosition(servoL);
            servoRight.setPosition(servoR);
            telemetry.addData("posLeft", servoLeft.getPosition());
            telemetry.addData("posRight", servoRight.getPosition());
            telemetry.update();
        }
    }
}
