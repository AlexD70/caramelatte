package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Arm;

@TeleOp
public class TestArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Controller ctrl1 = new Controller(gamepad1);

        waitForStart();

        while(opModeIsActive()){
            if(ctrl1.circle.isPressed()){
                arm.setPosition(Arm.ArmPositions.COLLECT);
            } else if (ctrl1.cross.isPressed()){
                arm.setPosition(Arm.ArmPositions.PLACE);
            }
             ctrl1.update();
        }
    }
}
