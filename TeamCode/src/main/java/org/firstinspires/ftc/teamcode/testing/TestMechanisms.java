package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;

@TeleOp(name = "Test Arm & Intake", group = "Test")
public class TestMechanisms extends LinearOpMode {
    Arm arm;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap, "arm");
        intake = new Intake(hardwareMap, "crsleft", "crsright", "servo");
        lifter = new Lifter(hardwareMap, "")

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            arm.update();
            tele
            if(gamepad1.circle) {
                arm.setArmTarget(Arm.ArmPositions.INIT);
            }

            if(gamepad1.cross){
                arm.setArmTarget(Arm.ArmPositions.PRELOAD_PLACE);
            }

            if(gamepad1.square){
                arm.setArmTarget(Arm.ArmPositions.PLACE);
            }
        }
    }
}
