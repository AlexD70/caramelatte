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
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){


            if(gamepad1.circle) {
                arm.setArmTarget(Arm.ArmPositions.INIT);
            }

            if(gamepad1.cross){
                arm.setArmTarget(Arm.ArmPositions.PRELOAD_PLACE);
            }

            if(gamepad1.square){
                arm.setArmTarget(Arm.ArmPositions.PLACE);
            }
            arm.update();
            arm.printDebug(telemetry);
            telemetry.addData("Arm position", arm.getArmPosition());
            telemetry.update();
        }
    }
}
