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
public class TestRedFar extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 2;//1- dreapta, 2- centru, 3- stanga

    public TrajectorySequence[] preloadAndCollect = new TrajectorySequence[4], toBackdropPreload = new TrajectorySequence[4], toStackCycle1 = new TrajectorySequence[4];
    public TrajectorySequence toBackdropCycle1[] = new TrajectorySequence[4], parking[] = new TrajectorySequence[4];

    public void buildTrajectories(int caz){
        Vector2d splineVector2d = new Vector2d();
        if(caz == 1){
            splineVector2d = new Vector2d(40, 52);
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(33, -22,Math.toRadians(150 )),Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(12, -51.5,Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(12, -50), () -> {
                        bot.intake.setPosition(0.55);
                        bot.intake.startCollect();
                    })
                    .strafeRight(3)
                    .strafeLeft(3)
                    .build();

            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(11, 25))
                    .splineToConstantHeading(splineVector2d, Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(20, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.7);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .splineToConstantHeading(new Vector2d(9, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(50, 42))
                    .build();
        } else if(caz == 2){
            splineVector2d = new Vector2d(35, 52);
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(33,-33, Math.toRadians(-50)),Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(13,-52.5, Math.toRadians(-90)))
                    .addDisplacementMarker(10, () -> {
                        bot.intake.setPosition(0.65);
                        bot.intake.startCollect();
                    })
                    .strafeRight(3)
                    .strafeLeft(3)
                    .build();

            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(11, 25))
                    .splineToConstantHeading(splineVector2d, Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(20, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.7);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .splineToConstantHeading(new Vector2d(9, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(50, 42))
                    .build();
        } else {
            splineVector2d = new Vector2d(35, 52);
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62, -34, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(33, -50.5, Math.toRadians(330)),Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(12.5, -52, Math.toRadians(-90)))
                    .addDisplacementMarker(12, () -> {
                        bot.intake.setPosition(0.65);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(11, 25))
                    .splineToConstantHeading(splineVector2d, Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(20, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.7);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .splineToConstantHeading(new Vector2d(9, 42), Math.toRadians(-90))
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


//        toStackCycle1 = bot.drive.trajectorySequenceBuilder(toBackdropPreload.end())
//                .setReversed(false)
//                .addDisplacementMarker(4, () -> {
//                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                    sleep(200);//300
//                    bot.arm.setPosition(0.96);
//                    bot.lift.setTarget(0);
//                })
//                .splineToLinearHeading(new Pose2d(11, 10, Math.toRadians(-90)), Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(11, -45, Math.toRadians(-90)))
//                .splineToConstantHeading(new Vector2d(15, -51), Math.toRadians(-90))
//                .addSpatialMarker(new Vector2d(15, -50), () -> {
//                    bot.intake.setPosition(0.72);
//                    bot.intake.startCollect();
//                })
//                .build();
//
//        toBackdropCycle1 = bot.drive.trajectorySequenceBuilder(toStackCycle1.end())
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(10, 20, Math.toRadians(-90)))
//                .splineToLinearHeading(new Pose2d(26, 57.5, Math.toRadians(-90)), Math.toRadians(90))
//                .addSpatialMarker(new Vector2d(22, 50), () -> {
//                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
//                    bot.lift.setTarget(2000);
//                    sleep(100);
//                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
//                })
//                .build();
//
//        parking = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
//                .splineToConstantHeading(new Vector2d(9, 42), Math.toRadians(-90))
//                .addDisplacementMarker(5, () -> {
//                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                    sleep(200);//300
//                    bot.arm.setPosition(0.96);
//                    bot.lift.setTarget(0);
//                })
//                .lineToConstantHeading(new Vector2d(50, 42))
//                .build();
    }

//    public void cycleOne(){
//        bot.drive.followTrajectorySequenceAsync(toStackCycle1);
//
//        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
//            bot.drive.update();
//            bot.lift.update();
//            telemetry.addData("err", bot.drive.getLastError());
//            telemetry.update();
//        }
//
//        new Thread(() -> {
//            sleep(1000);//1000
//            bot.outtake.catchPixels();
//            bot.intake.startEject();
//            sleep(1000);//1000
//            bot.intake.stopCollect();
//        }).start();
//        sleep(300);
//        bot.intake.setPosition(0.8);
//        sleep(950);
//
//        bot.intake.setPosition(0.3);
//
//
//        bot.drive.followTrajectorySequenceAsync(toBackdropCycle1);
//
//        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
//            bot.drive.update();
//            bot.lift.update();
//            bot.printDebug(telemetry);
//            telemetry.addData("er", bot.drive.getLastError());
//            telemetry.update();
//        }
//
////            sleep(200);//300
//        bot.outtake.dropBothPixels();
//
//
//        while (bot.lift.isBusy() && !isStopRequested()) {
//            bot.lift.update();
//            bot.printDebug(telemetry);
//            telemetry.update();
//        }
//
//    }
//
//    public void cycleTwo()
//    {
//        bot.drive.followTrajectorySequenceAsync(
//                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
//                        .setReversed(false)
//                        .addDisplacementMarker(4, () -> {
//                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                            sleep(300);//300
//                            bot.arm.setPosition(0.96);
//                            bot.lift.setTarget(0);
//                        })
//                        .splineToLinearHeading(new Pose2d(-11, 15, Math.toRadians(-90)), Math.toRadians(-90))
//                        .lineToLinearHeading(new Pose2d(-11,-52.5,Math.toRadians(-90)))
//                        .addSpatialMarker(new Vector2d(-11, -50), () -> {
//                            bot.intake.setPosition(0.9);
//                            bot.intake.startCollect();
//                        })
//                        .build()
//        );
//
//        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
//            bot.drive.update();
//            bot.lift.update();
//            telemetry.addData("err", bot.drive.getLastError());
//            telemetry.update();
//        }
//
//        sleep(1000);//500
//        bot.intake.setPosition(0.3);
//
//        new Thread(() -> {
//            sleep(1000);//1000
//            bot.outtake.catchPixels();
//            bot.intake.startEject();
//            sleep(500);//1000
//            bot.intake.stopCollect();
//        }).start();
//
//        bot.drive.followTrajectorySequenceAsync(
//                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
//                        .setReversed(true)
//                        .lineToLinearHeading(new Pose2d(-11, 15, Math.toRadians(-90)))
//                        .splineToLinearHeading(new Pose2d(-27, 57.5, Math.toRadians(-90)), Math.toRadians(90))
//                        .addSpatialMarker(new Vector2d(-20, 20), () -> {
//                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
//                            bot.lift.setTarget(2000);
//                            sleep(300);
//                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
//                        })
//                        .build()
//        );
//
//        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
////            sleep(300);//300
//        bot.arm.setPosition(0.96);
//        bot.lift.setTarget(0);
//
//        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
//            bot.drive.update();
//            bot.lift.update();
//            bot.printDebug(telemetry);
//            telemetry.addData("er", bot.drive.getLastError());
//            telemetry.update();
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.lift.setAuto();

        bot.outtake.dropRightPixel();

        buildTrajectories(1);
        buildTrajectories(2);
        buildTrajectories(3);

        telemetry.addLine("Cmon baby we know you can do it!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(62, -34, Math.toRadians(0)));

        bot.drive.followTrajectorySequenceAsync(preloadAndCollect[caz]);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(500);//500
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1000);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(1000);//1000
            bot.intake.stopCollect();
        }).start();

        sleep(500);

        bot.drive.followTrajectorySequenceAsync(toBackdropPreload[caz]);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(300);
        bot.outtake.dropBothPixels();

        while (bot.lift.isBusy() && !isStopRequested()) {
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

        telemetry.addLine("Helloo");
        telemetry.update();

//        cycleOne();
//
//        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
//            bot.drive.update();
//            bot.lift.update();
//            bot.printDebug(telemetry);
//            telemetry.addData("er", bot.drive.getLastError());
//            telemetry.update();
//        }
//
//        sleep(300);
//        bot.outtake.dropBothPixels();

        bot.drive.followTrajectorySequenceAsync(parking[caz]);



//        cycleTwo();
    }

}
