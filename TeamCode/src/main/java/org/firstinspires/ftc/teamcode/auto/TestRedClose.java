package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class TestRedClose extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 2;//1- stanga, 2- centru, 3- dreapta

    public void cycleOne()
    {
        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(new Pose2d(34, 44.5, Math.toRadians(90)))
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(200);//300
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(51, 15, Math.toRadians(90)), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(51, -35))
                        .splineToLinearHeading(new Pose2d(32, -60.5, Math.toRadians(90)), Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(32, -52), () -> {
                            bot.intake.setPosition(0.65);
                            bot.intake.startCollect();
                        })
                        .build()
        );

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(200);//500
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1000);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(1000);//1000
            bot.intake.stopCollect();
        }).start();

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(52, -37, Math.toRadians(90)), Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(52,15))
                        .addSpatialMarker(new Vector2d(29, 20), () -> {
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                            bot.lift.setTarget(2000);
                            sleep(100);
                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        })
                        .splineToLinearHeading(new Pose2d(29, 45, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

//            sleep(200);//300
        bot.outtake.dropBothPixels();


        while (bot.lift.isBusy() && !isStopRequested()) {
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

    }

    public void cycleTwo()
    {
        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(new Pose2d(29, 45, Math.toRadians(90)))
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(300);//300
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(50, 15, Math.toRadians(90)), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(50, -35))
                        .splineToLinearHeading(new Pose2d(28, -60.5, Math.toRadians(90)), Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(28, -55), () -> {
                            bot.intake.setPosition(0.9);
                            bot.intake.startCollect();
                        })
                        .build()
        );

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(200);//500
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1000);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(1000);//1000
            bot.intake.stopCollect();
        }).start();

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(90)), Math.toRadians(-90))
                        .lineToLinearHeading(new Pose2d(52,15,Math.toRadians(90)))
//                            .splineToLinearHeading(new Pose2d(-52, 15, Math.toRadians(-90)), Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(35, 20), () -> {
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                            bot.lift.setTarget(2000);
                            sleep(300);
                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        })
                        .splineToLinearHeading(new Pose2d(29, 45, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//            sleep(300);//300
        bot.arm.setPosition(0.96);
        bot.lift.setTarget(0);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.lift.setAuto();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(62, 12, Math.toRadians(-180)));
        if (caz == 1) {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(36, 13, Math.toRadians(-210)), Math.toRadians(0))
                            .setTangent(Math.toRadians(-(30 + 180)))
                            .splineToLinearHeading(new Pose2d(43, 45.5, Math.toRadians(90)), Math.toRadians(-90))
                            .addSpatialMarker(new Vector2d(43, 22), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.8);
                            })
//                        .lineTo(new Vector2d(-12, 25))
//                        .forward(80)
//                        .back(80)
//                        .strafeRight(20)
                            .build()
            );

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

            cycleOne();

            cycleTwo();
        }
        if (caz == 2) {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(30, 12, Math.toRadians(-150)), Math.toRadians(0))
                            .setTangent(Math.toRadians(-(30 + 180)))
                            .setVelConstraint(new TrajectoryVelocityConstraint() {
                                @Override
                                public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                                    return 50;
                                }
                            })
                            .splineToLinearHeading(new Pose2d(34, 45, Math.toRadians(90)), Math.toRadians(-90))
                            .resetVelConstraint()
                            .addSpatialMarker(new Vector2d(32, 20), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.8);
                            })
//                        .lineTo(new Vector2d(-12, 25))
//                        .forward(80)
//                        .back(80)
//                        .strafeRight(20)
                            .build()
            );

            while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
                bot.drive.update();
                bot.lift.update();
                bot.printDebug(telemetry);
                telemetry.addData("er", bot.drive.getLastError());
                telemetry.update();
            }

//            sleep(200);//300
            bot.outtake.dropBothPixels();


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
        if (caz == 3) {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(40, 0, Math.toRadians(-150)), Math.toRadians(0))
                            .setTangent(Math.toRadians(-(30 + 180)))
                            .splineToLinearHeading(new Pose2d(40, 45, Math.toRadians(90)), Math.toRadians(-90))
                            .addSpatialMarker(new Vector2d(30, 22), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.8);
                            })
//                        .lineTo(new Vector2d(-12, 25))
//                        .forward(80)
//                        .back(80)
//                        .strafeRight(20)
                            .build()
            );

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

            cycleOne();

            cycleTwo();
        }

        sleep(3000);
    }

}
