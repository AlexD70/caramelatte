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

@Autonomous(name = "far left blue", group = "testing")
public class TestCase3FarBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive rr = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Lifter lift = new Lifter(hardwareMap);

        waitForStart();

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-45, -6, Math.toRadians(30)))
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
                        .addSpatialMarker(new Vector2d(-30, -70), () -> {
                            lift.goToPos(1050);
                            arm.forceArmToPosition(1800);
                            intake.forceAngleServoPos(0.75);
                        })
                        .lineToLinearHeading(new Pose2d(-41, -60, Math.toRadians(90)))
                        .back(10)
                        //.strafeRight(63, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(20))
                        .lineToLinearHeading(new Pose2d(-21.1, -100.7, Math.toRadians(90)))
                        .build()
        );

        while(rr.isBusy()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }


        ElapsedTime lifterTimeout = new ElapsedTime();
        lifterTimeout.reset();
        while(arm.isArmBusy() && lifterTimeout.seconds() < 0.6){
            lift.update();
            arm.update(telemetry);
        }

        arm.setArmTarget(Arm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 2){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.forceArmToPosition(0);
        timer.reset();
        while(timer.seconds() < 1){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 2){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(20)
                        .build()
        );

        sleep(5000);

    }
}
