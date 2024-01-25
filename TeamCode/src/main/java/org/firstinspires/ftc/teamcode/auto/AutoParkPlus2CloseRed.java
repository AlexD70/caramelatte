package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;

@Autonomous(name = "2+P CLOSE RED", group = "auto")
public class AutoParkPlus2CloseRed extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    VoltageScaledArm arm;
    Lifter lift;
    HuskyLensDetection husky;
    ColorSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        rr = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new VoltageScaledArm(hardwareMap);
        lift = new Lifter(hardwareMap);
        husky = new HuskyLensDetection(hardwareMap, "husky");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

        HuskyLensDetection.RandomisationCase randomisationCase = HuskyLensDetection.RandomisationCase.UNKNOWN;

        while(!isStarted()){
            randomisationCase = husky.getCaseRedClose(telemetry);
            telemetry.addData("CASE ", randomisationCase);
            telemetry.update();
        }

        waitForStart();
        sensor.enableLed(false);

        int randomization = (int) Math.round(Math.random() * 2) - 3;

        if(randomisationCase != HuskyLensDetection.RandomisationCase.UNKNOWN){
            randomization = randomisationCase.val;
        }


        if (randomization == 1) { // STANGA RED
            leftRed();
        } else if (randomization == 0) { // CENTER RED
            centerRed();
        } else { // DREAPTA RED
            rightRed();
        }
    }


    public void leftRed() throws InterruptedException{ //TestCase1Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, 33, Math.toRadians(-90)), Math.toRadians(90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }

        intake.forceAngleServoPos(0.3);
        Thread.sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        Thread.sleep(500);

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-10, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1100);
                        })
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
        }

        while(lift.isBusy() && !isStopRequested()){
            lift.update();
        }

        intake.forceAngleServoPos(0.75);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
        );

        lift.goToPos(Lifter.LifterStates.DOWN);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(19)
                        .build()
        );
    }

    public void centerRed() throws InterruptedException { //TestCase2Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-25, 28, Math.toRadians(-90)), Math.toRadians(90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }


        intake.forceAngleServoPos(0.3);
        Thread.sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        Thread.sleep(500);

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-12.5, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1100);
                        })
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
        }


        while(lift.isBusy() && !isStopRequested()){
            lift.update();
        }

        intake.forceAngleServoPos(0.75);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
        );

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
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
    }

    public void rightRed() throws InterruptedException { // TestCase3Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, 13, Math.toRadians(-90)))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }


        intake.forceAngleServoPos(0.3);
        Thread.sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        Thread.sleep(500);

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-18.5, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1050);
                        })
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
        }


        while(lift.isBusy() && !isStopRequested()){
            lift.update();
        }

        intake.forceAngleServoPos(0.75);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
//        rr.followTrajectorySequence(
//                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
//        );

        lift.goToPos(Lifter.LifterStates.DOWN);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(25)
                        .build()
        );
    }
}
