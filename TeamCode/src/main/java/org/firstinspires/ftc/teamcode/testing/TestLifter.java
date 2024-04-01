package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Lifter;

@TeleOp
public class TestLifter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lifter lift = new Lifter(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        arm.setPosition(Arm.ArmPositions.PLACE);
        Controller ctrl1 = new Controller(gamepad1);

        waitForStart();

        while(opModeIsActive()){
            lift.update();
            ctrl1.update();
            if(ctrl1.dpadUp.isPressed()){
                lift.setTarget(2000);
            } else if(ctrl1.dpadDown.isPressed()){
                lift.setTarget(0);
            } else if(ctrl1.dpadLeft.isPressed()){
                lift.setTarget(1000);
            } else if(ctrl1.dpadRight.isPressed()){
                lift.setTarget(3000);
            }

            lift.printDebug(telemetry);
            telemetry.update();
        }
    }
}
