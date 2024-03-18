package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class TestBlueClose extends LinearOpMode {
    Robot bot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.lift.setAuto();

        waitForStart();

        if(isStopRequested()){
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(-62, 12, Math.toRadians(180)));

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(new Pose2d(-62, 12, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-30, 12, Math.toRadians(150)), Math.toRadians(0))
                        .setTangent(Math.toRadians(30 + 180))
                        .splineToLinearHeading(new Pose2d(-30, 45, Math.toRadians(-90)), Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(-30, 22), () -> {
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

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(300);
        bot.outtake.dropBothPixels();



        while(bot.lift.isBusy() && !isStopRequested()){
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

        telemetry.addLine("Helloo");
        telemetry.update();

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(new Pose2d(-35, 44, Math.toRadians(-90)))
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(300);
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(-52, 15, Math.toRadians(-90)), Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(-52, -35))
                        .splineToLinearHeading(new Pose2d(-29.5, -59, Math.toRadians(-90)), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-29, -52), () -> {
                            bot.intake.setPosition(0.6);
                            bot.intake.startCollect();
                        })
                        .build()
        );

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(500);
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1000);
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(1000);
            bot.intake.stopCollect();
        }).start();

        bot.drive.followTrajectorySequenceAsync(
               bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                       .setReversed(true)
                       .splineToLinearHeading(new Pose2d(-54, -40, Math.toRadians(-90)), Math.toRadians(90))
                       .lineToConstantHeading(new Vector2d(-54, 15))
                       .addSpatialMarker(new Vector2d(-35, 20), () -> {
                           bot.arm.setPosition(Arm.ArmPositions.PLACE);
                           bot.lift.setTarget(2000);
                           sleep(300);
                           bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                       })
                       .splineToLinearHeading(new Pose2d(-29, 44, Math.toRadians(-90)), Math.toRadians(90))
                       .build()
        );

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(300);
        bot.outtake.dropBothPixels();



        while(bot.lift.isBusy() && !isStopRequested()){
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(new Pose2d(-35, 44, Math.toRadians(-90)))
                        .setReversed(false)
                        .addDisplacementMarker(4, () -> {
                            bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                            bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                            sleep(300);
                            bot.arm.setPosition(0.96);
                            bot.lift.setTarget(0);
                        })
                        .splineToLinearHeading(new Pose2d(-52, 15, Math.toRadians(-90)), Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(-52, -35))
                        .splineToLinearHeading(new Pose2d(-29.2, -59, Math.toRadians(-90)), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-29, -52), () -> {
                            bot.intake.setPosition(0.8);
                            bot.intake.startCollect();
                        })
                        .build()
        );

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(500);
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1000);
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(1000);
            bot.intake.stopCollect();
        }).start();

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-52, -40, Math.toRadians(-90)), Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(-52, 15))
                        .addSpatialMarker(new Vector2d(-35, 20), () -> {
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                            bot.lift.setTarget(2000);
                            sleep(300);
                            bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        })
                        .splineToLinearHeading(new Pose2d(-29, 44, Math.toRadians(-90)), Math.toRadians(90))
                        .build()
        );

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }


        sleep(3000);
    }
}
