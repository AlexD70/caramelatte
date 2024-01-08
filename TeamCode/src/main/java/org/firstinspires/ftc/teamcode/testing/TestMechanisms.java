package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.MecanumDriveEx;

@TeleOp(name = "Test Teleop", group = "Test")
public class TestMechanisms extends LinearOpMode {
    Arm arm;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        MecanumDriveEx drive = new MecanumDriveEx(hardwareMap);
        Lifter lift = new Lifter(hardwareMap);

        waitForStart();

        arm.setArmTarget(Arm.ArmPositions.COLLECT);

        while(opModeIsActive() && !isStopRequested()){
            drive.setDrivePower(new Pose2d(-gamepad1.right_stick_x, gamepad1.left_stick_x, -gamepad1.left_stick_y));
            arm.update(telemetry);
            lift.update();

            if(gamepad2.circle) {
                lift.goToPos(Lifter.LifterStates.DOWN);
                arm.setArmTarget(Arm.ArmPositions.COLLECT);
                intake.forceAngleServoPos(0.55);
            }

            if(gamepad2.cross){
                arm.setArmTarget(Arm.ArmPositions.PRELOAD_PLACE);
                intake.forceAngleServoPos(0.9);
            }

            if(gamepad2.square){
                lift.goToPos(Lifter.LifterStates.HIGH);
            }

            if(gamepad2.triangle){
                lift.goToPos(Lifter.LifterStates.DOWN);
            }

//            if(gamepad1.left_bumper){
//                intake.startCollect();
//                intake.forceAngleServoPos(0.54);
//            }
//
//            if(gamepad1.right_bumper){
//                intake.stopCollect();
//                intake.forceAngleServoPos(1);
//            }
//
//            if(gamepad1.dpad_down){
//                intake.dropPixel();
//            }
            if(gamepad1.left_trigger > 0.3){
                intake.setCRSPowers(.5);
            } else if (gamepad1.right_trigger > 0.3){
                intake.setCRSPowers(-0.5);
            } else {
                intake.setCRSPowers(0);
            }

            arm.printDebug(telemetry);
            lift.printDebug(telemetry);
            telemetry.update();
        }
    }
}
