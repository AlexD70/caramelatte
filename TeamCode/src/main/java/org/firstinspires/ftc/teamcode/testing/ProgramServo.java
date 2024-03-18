package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ProgramServo extends LinearOpMode {
    public static double servoL = 1d, servoR = 0.3d, rotat = 0.468d;//pozitie mijloc 0.468d

    // 0.52 - rotation servo (middle position)
    // 0.65 - gear servo collect position
    // 0.15 - gear servo place position
    // 0.4 - avion armat
    // 0.6 lansare avion

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoLeft = hardwareMap.get(Servo.class, "launcher");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            servoLeft.setPosition(servoL);
//            servoRight.setPosition(servoR);
            telemetry.addData("posLeft", servoLeft.getPosition());
//            telemetry.addData("posRight", servoRight.getPosition());
            telemetry.update();
        }
    }
}
