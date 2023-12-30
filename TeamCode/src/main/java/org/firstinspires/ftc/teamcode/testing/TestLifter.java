package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.MecanumDriveEx;
import org.firstinspires.ftc.teamcode.util.PlaneLauncher;

@TeleOp(name = "Test Lifter & Drive", group = "Test")
public class TestLifter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lifter lift = new Lifter(hardwareMap);
        PlaneLauncher launcher = new PlaneLauncher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        MecanumDriveEx drive = new MecanumDriveEx(hardwareMap);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            drive.setDrivePower(new Pose2d(-gamepad1.right_stick_x, gamepad1.left_stick_x, -gamepad1.left_stick_y));

            if(gamepad1.square){
                lift.goToPos(1500);
            }

            if(gamepad1.circle){
                lift.goToPos(0);
            }

            if(gamepad1.triangle){
                lift.goToPos(1000);
            }

            if(gamepad1.left_bumper){
                intake.startCollect();
            }

            if(gamepad1.right_bumper){
                intake.stopCollect();
            }

            if(gamepad1.dpad_down){
                //intake.toggleAngleAdjustPos();
            }

            telemetry.addData("target:", lift.target);
            telemetry.addData("last target:", lift.lastTarget);
            telemetry.addData("position:", lift.currentPosition);
            telemetry.addData("power:", lift.power);
            telemetry.update();
            lift.update();
        }
    }
}
