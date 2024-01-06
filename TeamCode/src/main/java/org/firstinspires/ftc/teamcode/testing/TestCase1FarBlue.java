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

@Autonomous(name = "far right blue", group = "testing")
public class TestCase1FarBlue extends LinearOpMode {
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
                        .splineToLinearHeading(new Pose2d(-15, -14.8, Math.toRadians(90)), Math.toRadians(-90))
                        .back(25)
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
                        .back(20)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(0, -79), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-5.7, -100), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-3, -90), () -> {
                            lift.goToPos(1050);
                        })
                        .build()
        );

        while(rr.isBusy()){
            rr.update();
            lift.update();
        }


        ElapsedTime lifterTimeout = new ElapsedTime();
        lifterTimeout.reset();
        while(lift.isBusy() && lifterTimeout.seconds() < 0.6){
            lift.update();
        }

        intake.forceAngleServoPos(0.8);
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
                        .strafeLeft(30)
                        .build()
        );

        sleep(5000);

    }
}
