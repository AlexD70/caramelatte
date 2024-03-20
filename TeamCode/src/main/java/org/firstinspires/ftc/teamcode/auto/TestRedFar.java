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

import kotlin.math.UMathKt;

@Autonomous
public class TestRedFar extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 3;//1- stanga, 2- centru, 3- dreapta

    public void cycleOne(){
        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(200);//300
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(11, 10, Math.toRadians(90)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(11, -51,Math.toRadians(90)))
                        .addSpatialMarker(new Vector2d(12, -48), () -> {
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
                        .lineToLinearHeading(new Pose2d(11,10,Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(26, 57.5, Math.toRadians(90)), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(20, 20), () -> {
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                            bot.lift.setTarget(2000);
                            sleep(100);
                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        })
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
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(300);//300
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(11, 15, Math.toRadians(90)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(11,-52.5,Math.toRadians(90)))
                        .addSpatialMarker(new Vector2d(11, -50), () -> {
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
                        .lineToLinearHeading(new Pose2d(11,15,Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(27, 57.5, Math.toRadians(90)), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(20, 20), () -> {
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                            bot.lift.setTarget(2000);
                            sleep(300);
                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        })
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

        bot.drive.setPoseEstimate(new Pose2d(62, -34, Math.toRadians(-180)));

        if(caz == 1)
        {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(34,-22,Math.toRadians(-210)),Math.toRadians(0))
                            .addDisplacementMarker(4, () -> {
                                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                                bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                                bot.outtake.dropBothPixels();
                                sleep(200);
                                bot.arm.setPosition(0.96);
                                bot.lift.setTarget(0);
                            })
                            .lineToLinearHeading(new Pose2d(12,-51.5,Math.toRadians(-270)))
                            .addSpatialMarker(new Vector2d(-12,-50), () -> {
                                bot.intake.setPosition(0.6);
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
                            .back(80)
                            .splineTo(new Vector2d(37, 57.5), Math.toRadians(-90))
                            .addSpatialMarker(new Vector2d(20,20), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.52);
                            })
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
                bot.outtake.update();
                bot.lift.update();
                bot.printDebug(telemetry);
                telemetry.update();
            }

            telemetry.addLine("Helloo");
            telemetry.update();

            cycleOne();

            cycleTwo();
        }
        if(caz == 2)
        {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(37,-33,Math.toRadians(-210)),Math.toRadians(0))
                            .addDisplacementMarker(4, () -> {
                                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                                bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                                bot.outtake.dropBothPixels();
                                sleep(200);
                                bot.arm.setPosition(0.96);
                                bot.lift.setTarget(0);
                            })
                            .lineToLinearHeading(new Pose2d(12,-52.5,Math.toRadians(-270)))
                            .addSpatialMarker(new Vector2d(12,-50), () -> {
                                bot.intake.setPosition(0.6);
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
                            .back(80)
                            .splineTo(new Vector2d(33, 57.5), Math.toRadians(-90))
                            .addSpatialMarker(new Vector2d(20,20), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.52);
                                sleep(300);
                                bot.outtake.dropBothPixels();
                            })
                            .build()
            );

            while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
                bot.drive.update();
                bot.lift.update();
                bot.printDebug(telemetry);
                telemetry.addData("er", bot.drive.getLastError());
                telemetry.update();
            }

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
        if(caz == 3)
        {
            bot.drive.followTrajectorySequenceAsync(
                    bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(-180)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(38,-50.5,Math.toRadians(-210)),Math.toRadians(0))
                            .addDisplacementMarker(4, () -> {
                                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                                bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                                bot.outtake.dropBothPixels();
                                sleep(200);
                                bot.arm.setPosition(0.96);
                                bot.lift.setTarget(0);
                            })
                            .lineToLinearHeading(new Pose2d(11,-52,Math.toRadians(-270)))
                            .addSpatialMarker(new Vector2d(11,-50), () -> {
                                bot.intake.setPosition(0.7);
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

            sleep(500);//500
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
                            .lineToLinearHeading(new Pose2d(11,30, Math.toRadians(90)))
                            .setTangent(-(30 + 180))
                            .splineToLinearHeading(new Pose2d(26, 55,Math.toRadians(90)), Math.toRadians(90))
                            .addSpatialMarker(new Vector2d(20,20), () -> {
                                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                                bot.lift.setTarget(1000);
                                sleep(300);
                                bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                                bot.outtake.rotateToAngleManual(0.52);
                            })
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
    }
}
