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

@Autonomous(name = "far center red", group = "testing")
public class TestCase2FarRed extends LinearOpMode {
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
                        .back(52)
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
                rr.trajectorySequenceBuilder(new Pose2d(-52, 0, 0))
                        .forward(3)
                        .addSpatialMarker(new Vector2d(-30, 70), () -> {
                            lift.goToPos(1050);
                            arm.forceArmToPosition(1800);
                            intake.forceAngleServoPos(0.75);
                        })
                        .lineToLinearHeading(new Pose2d(-43, 60, Math.toRadians(-90)))
                        .back(10)
                        //.strafeRight(63, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(20))
                        .setTangent(Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-17.3, 100.7, Math.toRadians(-90)))
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

        //intake.forceAngleServoPos(0.7);
        //arm.setArmTarget(Arm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 0.5){
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
        while(timer.seconds() < 2){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeRight(25)
                        .build()
        );

        sleep(5000);

    }
}
