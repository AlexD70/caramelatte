package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;

@Autonomous(name = "test auto left blue", group = "testing")
public class TestCase3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive rr = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Lifter lift = new Lifter(hardwareMap);

        telemetry.setAutoClear(false);
        waitForStart();

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, -13, Math.toRadians(90)))
                        .build()
        );

        while(rr.isBusy()){
            rr.update();
        }


        intake.forceAngleServoPos(0.5);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        try{
            Thread.sleep(500);
        } catch (InterruptedException e){
            e.printStackTrace();
        }

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-18, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
                            lift.goToPos(1050);
                        })
                        .build()
        );

        while(rr.isBusy()){
            rr.update();
            lift.update();
        }


        while(lift.isBusy()){
            lift.update();
        }

        intake.forceAngleServoPos(0.8);
        arm.setArmTarget(Arm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 3){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.forceArmToPosition(-10);
        timer.reset();
        while(timer.seconds() < 1){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 3){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeRight(26)
                        .build()
        );

        sleep(5000);
    }
}
