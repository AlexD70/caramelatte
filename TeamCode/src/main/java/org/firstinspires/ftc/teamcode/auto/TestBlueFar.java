package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.BluePipeline;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import kotlin.math.UMathKt;

@Autonomous
public class TestBlueFar extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 3;//1- stanga, 2- centru, 3- dreapta

    OpenCvWebcam webcam;
    BluePipeline pipeline = new BluePipeline();
    boolean cameraOK = true;

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000); //3000
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try {
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera failed to open:", errorCode);
                telemetry.update();
                cameraOK = false;
            }
        });
    }

    public TrajectorySequence preloadAndCollect[] = new TrajectorySequence[4], toBackdropPreload[] = new TrajectorySequence[4], toStackCycle1[] = new TrajectorySequence[4];
    public TrajectorySequence toBackdropCycle1[] = new TrajectorySequence[4], parking[] = new TrajectorySequence[4];

    public void buildTrajectories(int caz){
        Vector2d splineVector2d = new Vector2d();
        if(caz == 1){
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-62,-34,Math.toRadians(180)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-34, -22,Math.toRadians(210)),Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-10, -53.5,Math.toRadians(270)))
                    .addTemporalMarker(3, () -> {
                        bot.intake.setPosition(0.58);
                        bot.intake.startCollect();
                    })
                    .build();
            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(-11, 25))
                    .splineToLinearHeading(new Pose2d(-29, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(-20, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.2);
                    })
                    .build();

            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(4, () -> {
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(-9, 10, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-9, -51, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-9, -50), () -> {
                        bot.intake.setPosition(0.72);
                        bot.intake.startCollect();
                    })
                    .strafeRight(3)
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-10,-30,Math.toRadians(-90)),Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-10, 20, Math.toRadians(-90)))
                    .splineToLinearHeading(new Pose2d(-26, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(-22, 50), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(2000);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .splineToConstantHeading(new Vector2d(-3, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(-3, 58))
                    .build();
        } else if(caz == 2){
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-62,-34,Math.toRadians(180)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-33,-33, Math.toRadians(210)),Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-10,-53.5, Math.toRadians(270)))
                    .addTemporalMarker(3, () -> {
                        bot.intake.setPosition(0.58);
                        bot.intake.startCollect();
                    })
                    .build();
            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(-11, 25))
                    .splineToLinearHeading(new Pose2d(-24, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(-20, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.2);

                    })
                    .build();

            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(4, () -> {
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(-9, 10, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-9, -51, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-9, -50), () -> {
                        bot.intake.setPosition(0.72);
                        bot.intake.startCollect();
                    })
                    .strafeRight(8)
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-10,-30,Math.toRadians(-90)),Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-10, 20, Math.toRadians(-90)))
                    .splineToLinearHeading(new Pose2d(-20, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(-18, 50), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(2000);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .splineToConstantHeading(new Vector2d(-3, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(-3, 58))
                    .build();
        } else {
            preloadAndCollect[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-34, -50, Math.toRadians(210)),Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(-12, -53.5, Math.toRadians(270)))
                    .addTemporalMarker(3, () -> {
                        bot.intake.setPosition(0.58);
                        bot.intake.startCollect();
                    })
                    .build();
            toBackdropPreload[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(-11, -50, Math.toRadians(-90)))
                    .setReversed(true)
                    .lineToConstantHeading(new Vector2d(-11, 25))
                    .addSpatialMarker(new Vector2d(-11, 25), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.7);
                    })
                    .splineToLinearHeading(new Pose2d(-23, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();

            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(toBackdropPreload[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(4, () -> {
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(-9, 10, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-9, -51, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(-9, -50), () -> {
                        bot.intake.setPosition(0.7);
                        bot.intake.startCollect();
                    })
                    .strafeRight(8)
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-9,-40,Math.toRadians(-90)),Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-9, 20, Math.toRadians(-90)))
                    .splineToLinearHeading(new Pose2d(-20, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(-18, 50), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .splineToConstantHeading(new Vector2d(-3, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(-3, 56))
                    .build();
        }

    }

    public void cycleOne(){
        bot.drive.followTrajectorySequenceAsync(toStackCycle1[caz]);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        new Thread(() -> {
            sleep(1500);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(500);//1000
            bot.intake.stopCollect();
        }).start();
        sleep(300);
        bot.intake.setPosition(0.8);
        sleep(950);

        bot.intake.setPosition(0.3);


        bot.drive.followTrajectorySequenceAsync(toBackdropCycle1[caz]);

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
                        .splineToLinearHeading(new Pose2d(-11, 15, Math.toRadians(-90)), Math.toRadians(-90))
                        .lineToLinearHeading(new Pose2d(-11,-52.5,Math.toRadians(-90)))
                        .addSpatialMarker(new Vector2d(-11, -50), () -> {
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

        sleep(1000);//500
        bot.intake.setPosition(0.3);

        new Thread(() -> {
            sleep(1500);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(500);//1000
            bot.intake.stopCollect();
        }).start();

        sleep(1000);

        bot.drive.followTrajectorySequenceAsync(
                bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-11, 15, Math.toRadians(-90)))
                        .splineToLinearHeading(new Pose2d(-27, 57.5, Math.toRadians(-90)), Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(-20, 20), () -> {
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
        bot.outtake.dropBothPixels();

        initDetection();

        pipeline.startDetection(false);

        if (cameraOK) {
            telemetry.addLine("Webcam Ok");
        } else {
            telemetry.addLine("Webcam failed, please RESTART!");
            telemetry.update();
            sleep(1000);
        }

        buildTrajectories(1);
        buildTrajectories(2);
        buildTrajectories(3);

        telemetry.addLine("Cmon baby we know you can do it!");
        telemetry.update();
        telemetry.setAutoClear(false);

        while(!isStarted() && !isStopRequested()){
            telemetry.addData("case", pipeline.getCase());
        }

        waitForStart();
        int temp = pipeline.getCase();
        if(temp == 0){
            caz = 2;
        } else if (temp == -2){
            caz = 1;
        } else if (temp == 1) {
            caz = 3;
        }

        pipeline.killThis();
        webcam.stopStreaming();

        if (isStopRequested()) {
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(-62, -34, Math.toRadians(180)));

        bot.drive.followTrajectorySequenceAsync(preloadAndCollect[caz]);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            telemetry.addData("err", bot.drive.getLastError());
            telemetry.update();
        }

        new Thread(() -> {
            sleep(1500);//1000
            bot.outtake.catchPixels();
            bot.intake.startEject();
            sleep(500);//1000
            bot.intake.stopCollect();
        }).start();
        sleep(300);
        bot.intake.setPosition(0.8);
        sleep(950);

        bot.intake.setPosition(0.3);

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

        cycleOne();

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        sleep(300);
        bot.outtake.dropBothPixels();

        bot.drive.followTrajectorySequenceAsync(parking[caz]);

        while((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()){
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.update();
        }

//        cycleTwo();
    }
}

