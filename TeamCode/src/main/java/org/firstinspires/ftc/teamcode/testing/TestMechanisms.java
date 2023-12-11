package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;

@TeleOp(name = "Test Arm & Intake", group = "Test")
public class TestMechanisms extends LinearOpMode {
    Arm arm;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap, "arm");
        intake = new Intake(hardwareMap, "crsleft", "crsright", "servo");

        while(opModeIsActive()){
            if(gamepad1.circle){
                arm.forceArmToPosition(30);
            }

            if(gamepad1.cross){
                arm.forceArmToPosition(-30);
            }

            if(gamepad1.square){
                arm.setArmTarget(Arm.ArmPositions.INIT);
            }
        }
    }
}
