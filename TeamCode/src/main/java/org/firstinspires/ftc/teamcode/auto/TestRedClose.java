package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class TestRedClose extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 3;//1- dreapta, 2- centru, 3- stanga

    public TrajectorySequence preloads, toStackCycle1, toBackdropCycle1;
    public TrajectorySequence toStackCycle2, toBackdropCycle2, parking;

    public void buildTrajectories(int caz){
        if(caz == 1){ // RIGHT
            preloads = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36, 13, Math.toRadians(150)), Math.toRadians(180))
                    .setTangent(Math.toRadians(30 + 180))
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(43, 46, Math.toRadians(90)), Math.toRadians(-90))
                    .resetAccelConstraint()
                    .addSpatialMarker(new Vector2d(43, 22), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.8);
                    })
                    .build();
        } else if (caz == 2) { // CENTER
            preloads = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(30, 12, Math.toRadians(210)), Math.toRadians(180))
                    .setTangent(Math.toRadians(30 + 180))
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(34, 45, Math.toRadians(90)), Math.toRadians(-90))
                    .resetAccelConstraint()
                    .addSpatialMarker(new Vector2d(32, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.8);
                    })
                    .build();
        } else {
            preloads = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36, 0, Math.toRadians(210)), Math.toRadians(30))
                    .setTangent(Math.toRadians(30 + 180))
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(30, 45.5, Math.toRadians(90)), Math.toRadians(-90))
                    .resetAccelConstraint()
                    .addSpatialMarker(new Vector2d(30, 22), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.8);
                    })
                    .build();
        }

        double deltaX = (caz == 1)?(2):((caz == 2)?(0):(1));
        toStackCycle1 = bot.drive.trajectorySequenceBuilder(new Pose2d(34, 44.5, Math.toRadians(-90)))
                .setReversed(false)
                .addDisplacementMarker(4, () -> {
                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                    sleep(200);//300
                    bot.arm.setPosition(0.96);
                    bot.lift.setTarget(0);
                })
                .splineToLinearHeading(new Pose2d(52 + deltaX, 15, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(52 + deltaX, -35))
                .splineToLinearHeading(new Pose2d(34 + deltaX, -62, Math.toRadians(-90)), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(34 + deltaX, -52), () -> {
                    bot.intake.setPosition(0.56);
                    bot.intake.startCollect();
                })
                .build();

        toBackdropCycle1 = bot.drive.trajectorySequenceBuilder(toStackCycle1.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(52 + deltaX, -37, Math.toRadians(-90)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(52 + deltaX, 15))
                .addSpatialMarker(new Vector2d(-29, 20), () -> {
                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
                    bot.lift.setTarget(1500);
                    sleep(100);
                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                })
                .splineToLinearHeading(new Pose2d(29, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                .build();



        toStackCycle2 = bot.drive.trajectorySequenceBuilder(toBackdropCycle1.end())
                .setReversed(false)
                .addDisplacementMarker(4, () -> {
                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                    sleep(300);//300
                    bot.arm.setPosition(0.96);
                    bot.lift.setTarget(0);
                })
                .splineToLinearHeading(new Pose2d(48 + deltaX, 15, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(48 + deltaX, -35))
                .splineToLinearHeading(new Pose2d(28 + deltaX, -61, Math.toRadians(-90)), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(28 + deltaX, -55), () -> {
                    bot.intake.setPosition(0.7);
                    bot.intake.startCollect();
                })
                .build();

        toBackdropCycle2 = bot.drive.trajectorySequenceBuilder(toStackCycle2.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(48 + deltaX, -36, Math.toRadians(-90)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(48 + deltaX, 15, Math.toRadians(-90)))
                .addSpatialMarker(new Vector2d(35, 20), () -> {
                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
                    bot.lift.setTarget(1500);
                    sleep(300);
                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                })
                .splineToLinearHeading(new Pose2d(29 - ((caz == 3)?(-4):(deltaX)), 45.5, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        parking = bot.drive.trajectorySequenceBuilder(toBackdropCycle2.end())
                .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
                .addDisplacementMarker(5, () -> {
                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                    sleep(200);//300
                    bot.arm.setPosition(0.96);
                    bot.lift.setTarget(0);
                })
                .lineToConstantHeading(new Vector2d(50, 42))
                .build();
    }

    public void cycleOne()
    {
        bot.drive.followTrajectorySequenceAsync(toStackCycle1);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        new Thread(() -> {
            sleep(1000);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(500);//1000
            bot.intake.stopCollect();
        }).start();
        sleep(300);
        bot.intake.setPosition(0.64);
        sleep(950);

        bot.intake.setPosition(0.3);


        bot.drive.followTrajectorySequenceAsync(toBackdropCycle1);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        bot.outtake.dropBothPixels();
        sleep(300);
    }

    public void cycleTwo()
    {
        bot.drive.followTrajectorySequenceAsync(toStackCycle2);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        new Thread(() -> {
            sleep(1000);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(500);//1000
            bot.intake.stopCollect();
        }).start();
        sleep(300);
        bot.intake.setPosition(0.75);
        sleep(950);

        bot.intake.setPosition(0.3);

        bot.drive.followTrajectorySequenceAsync(toBackdropCycle2);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        bot.outtake.dropBothPixels();
        sleep(300);

        bot.drive.followTrajectorySequenceAsync(parking);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.lift.update();
            bot.drive.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.lift.setAuto();

        buildTrajectories(caz);

        telemetry.addLine("OK. Start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(62, 12, Math.toRadians(0)));
        bot.drive.followTrajectorySequenceAsync(preloads);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        bot.outtake.dropBothPixels();
        sleep(300);

        while (bot.lift.isBusy() && !isStopRequested()) {
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

        telemetry.addLine("Helloo");
        telemetry.update();

        cycleOne();

        cycleTwo();
    }

}
