package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;

@Autonomous(name = "2+P CLOSE BLUE", group = "auto")
public class AutoParkPlus2CloseBlue extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    Arm arm;
    Lifter lift;
    HuskyLensDetection husky;

    @Override
    public void runOpMode() throws InterruptedException {
        rr = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        lift = new Lifter(hardwareMap);
        husky = new HuskyLensDetection(hardwareMap, "husky");

        HuskyLensDetection.RandomisationCase randomisationCase = HuskyLensDetection.RandomisationCase.UNKNOWN;

        while(!isStarted()){
            randomisationCase = husky.getRandomisationCase(1);
            telemetry.addData("CASE ", randomisationCase);
            telemetry.update();
        }

        waitForStart();

        int randomization = (int) Math.round(Math.random() * 2) - 3;

        if(randomisationCase != HuskyLensDetection.RandomisationCase.UNKNOWN){
            randomization = randomisationCase.val;
        }

        if (randomization == -1) { // STANGA BLUE
            blueLeft();
        } else if (randomization == 0) { // CENTER BLUE
            centerBlue();
        } else { // DREAPTA BLUE
            rightBlue();
        }
    }


    public void blueLeft(){ // TestAxes.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, -33, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
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
                        .lineToConstantHeading(new Vector2d(-9, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
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
        arm.setArmTarget(Arm.ArmPositions.PLACE);
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

        arm.forceArmToPosition(-10);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeRight(20)
                        .build()
        );
    }

    public void centerBlue(){ //TestCase2.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-25, -28, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
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
                        .lineToConstantHeading(new Vector2d(-12, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
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
        arm.setArmTarget(Arm.ArmPositions.PLACE);
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

        arm.forceArmToPosition(-10);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeRight(22)
                        .build()
        );

    }

    public void rightBlue(){ //TestCase2.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, -13.5, Math.toRadians(90)))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
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
                        .lineToConstantHeading(new Vector2d(-18, -45.4))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
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
        arm.setArmTarget(Arm.ArmPositions.PLACE);
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

        arm.forceArmToPosition(-10);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 3 && !isStopRequested()){
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

    }
}
