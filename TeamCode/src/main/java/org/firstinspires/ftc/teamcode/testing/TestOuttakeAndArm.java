package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Outtake;

@TeleOp
public class TestOuttakeAndArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Controller ctrl1 = new Controller(gamepad1);

        waitForStart();

        while(opModeIsActive()){
            ctrl1.update();
            arm.update();
            outtake.update();

            if(ctrl1.cross.isPressed()){
                outtake.gearToPos(Outtake.GearStates.COLLECT);
                sleep(300);
                arm.setPosition(Arm.ArmPositions.COLLECT);
            }

            if(ctrl1.circle.isPressed()){
                outtake.gearToPos(Outtake.GearStates.PLACE);
                sleep(300);
                arm.setPosition(Arm.ArmPositions.PLACE);
            }
        }
    }
}
