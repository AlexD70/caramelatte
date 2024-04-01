package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.RedBluePipeline;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestRedClosePreloadOnCenternLeft extends LinearOpMode {
    Robot bot = new Robot();
    int caz = 1;//1- dreapta, 2- centru, 3- stanga

    OpenCvWebcam webcam;
    RedBluePipeline pipeline = new RedBluePipeline();
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


    public TrajectorySequence preloads[] = new TrajectorySequence[4], toStackCycle1[] = new TrajectorySequence[4], toBackdropCycle1[] = new TrajectorySequence[4];
    public TrajectorySequence toStackCycle2[] = new TrajectorySequence[4], toBackdropCycle2[] = new TrajectorySequence[4], parking[] = new TrajectorySequence[4];

    public void buildTrajectories(int caz){
        if(caz == 1){ // RIGHT
            preloads[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36, 13, Math.toRadians(-30)), Math.toRadians(180))
                    .setTangent(Math.toRadians(30))
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(43, 46, Math.toRadians(-90)), Math.toRadians(90))
                    .resetAccelConstraint()
                    .addSpatialMarker(new Vector2d(53, 30), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.8);
                    })
                    .build();
            Pose2d afterPreloadPose = new Pose2d(43, 46, Math.toRadians(-90));
            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(afterPreloadPose)
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(58, -35))
                    .splineToLinearHeading(new Pose2d(38, -62, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(32, -52), () -> {
                        bot.intake.setPosition(0.65);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(55.5, -37, Math.toRadians(-90)), Math.toRadians(90))
                    .resetAccelConstraint()
                    .lineToConstantHeading(new Vector2d(55.5, 15))
                    .addSpatialMarker(new Vector2d(29, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(35, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();



            toStackCycle2[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(300);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(56, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(56, -35))
                    .splineToLinearHeading(new Pose2d(37.5, -61, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(34, -55), () -> {
                        bot.intake.setPosition(0.7);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle2[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle2[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(57.5, -36, Math.toRadians(-90)), Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(57.5, 15, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(35, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(36, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle2[caz].end())
                    .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(60, 42))
                    .build();
        }
        else if (caz == 2) { // CENTER
            preloads[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(30, 12, Math.toRadians(30)), Math.toRadians(180))
                    .setTangent(Math.toRadians(30))
                    .splineToLinearHeading(new Pose2d(38, 44.5, Math.toRadians(270)), Math.toRadians(90))
                    .addSpatialMarker(new Vector2d(50, 40), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.2);
                    })
                    .build();
            Pose2d afterPreloadPose =new Pose2d(34, 45, Math.toRadians(270));
            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(afterPreloadPose)
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(58.5, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(58.5, -35))
                    .splineToLinearHeading(new Pose2d(37.5, -62, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(32, -52), () -> {
                        bot.intake.setPosition(0.68);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(55.5, -37, Math.toRadians(-90)), Math.toRadians(90))
                    .resetAccelConstraint()
                    .lineToConstantHeading(new Vector2d(55.5, 15))
                    .addSpatialMarker(new Vector2d(29, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(35, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();



            toStackCycle2[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(300);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(56, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(56, -35))
                    .splineToLinearHeading(new Pose2d(31.5, -61, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(28, -55), () -> {
                        bot.intake.setPosition(0.7);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle2[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle2[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(56, -36, Math.toRadians(-90)), Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(56, 15, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(35, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(36, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle2[caz].end())
                    .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(60, 42))
                    .build();
        } else {
            preloads[caz] = bot.drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(36, 0, Math.toRadians(30)), Math.toRadians(210))
                    .setTangent(Math.toRadians(30))
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(30, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .resetAccelConstraint()
                    .addSpatialMarker(new Vector2d(48, 32), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1000);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        bot.outtake.rotateToAngleManual(0.8);
                    })
                    .build();
            Pose2d afterPreloadPose = new Pose2d(30, 45.5, Math.toRadians(-90));
            toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(afterPreloadPose)
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(58, -35))
                    .splineToLinearHeading(new Pose2d(36, -62, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(32, -52), () -> {
                        bot.intake.setPosition(0.6);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(55.5, -37, Math.toRadians(-90)), Math.toRadians(90))
                    .resetAccelConstraint()
                    .lineToConstantHeading(new Vector2d(55.5, 15))
                    .addSpatialMarker(new Vector2d(29, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(100);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(35, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();



            toStackCycle2[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
                    .setReversed(false)
                    .addDisplacementMarker(6, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(300);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .splineToLinearHeading(new Pose2d(56, 15, Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToConstantHeading(new Vector2d(56, -35))
                    .splineToLinearHeading(new Pose2d(35, -61, Math.toRadians(-90)), Math.toRadians(-90))
                    .addSpatialMarker(new Vector2d(34, -55), () -> {
                        bot.intake.setPosition(0.7);
                        bot.intake.startCollect();
                    })
                    .build();

            toBackdropCycle2[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle2[caz].end())
                    .setReversed(true)
                    .setAccelConstraint(new ProfileAccelerationConstraint(25))
                    .splineToLinearHeading(new Pose2d(55, -36, Math.toRadians(-90)), Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(55, 15, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(35, 20), () -> {
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(1500);
                        sleep(300);
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    })
                    .splineToLinearHeading(new Pose2d(36, 45.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();

            parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle2[caz].end())
                    .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(60, 42))
                    .build();
        }
        }
//        Pose2d afterPreloadPose = (caz == 1)?(new Pose2d(43, 46, Math.toRadians(-90))):(caz == 2)?(new Pose2d(34, 45, Math.toRadians(270))):(new Pose2d(30, 45.5, Math.toRadians(-90)));
//        double deltaX = (caz == 1)?(6):((caz == 2)?(0):(6));
//        toStackCycle1[caz] = bot.drive.trajectorySequenceBuilder(afterPreloadPose)
//                .setReversed(false)
//                .addDisplacementMarker(6, () -> {
//                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                    sleep(200);//300
//                    bot.arm.setPosition(0.96);
//                    bot.lift.setTarget(0);
//                })
//                .splineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-90)), Math.toRadians(-90))
//                .lineToConstantHeading(new Vector2d(58, -35))
//                .splineToLinearHeading(new Pose2d(37.5, -62, Math.toRadians(-90)), Math.toRadians(-90))
//                .addSpatialMarker(new Vector2d(32, -52), () -> {
//                    bot.intake.setPosition(0.6);
//                    bot.intake.startCollect();
//                })
//                .build();
//
//        toBackdropCycle1[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle1[caz].end())
//                .setReversed(true)
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//                .splineToLinearHeading(new Pose2d(55.5, -37, Math.toRadians(-90)), Math.toRadians(90))
//                .resetAccelConstraint()
//                .lineToConstantHeading(new Vector2d(55.5, 15))
//                .addSpatialMarker(new Vector2d(29, 20), () -> {
//                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
//                    bot.lift.setTarget(1500);
//                    sleep(100);
//                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
//                })
//                .splineToLinearHeading(new Pose2d(35, 45.5, Math.toRadians(-90)), Math.toRadians(90))
//                .build();
//
//
//
//        toStackCycle2[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle1[caz].end())
//                .setReversed(false)
//                .addDisplacementMarker(6, () -> {
//                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                    sleep(300);//300
//                    bot.arm.setPosition(0.96);
//                    bot.lift.setTarget(0);
//                })
//                .splineToLinearHeading(new Pose2d(50 + deltaX, 15, Math.toRadians(-90)), Math.toRadians(-90))
//                .lineToConstantHeading(new Vector2d(50 + deltaX, -35))
//                .splineToLinearHeading(new Pose2d(30.5 + deltaX, -61, Math.toRadians(-90)), Math.toRadians(-90))
//                .addSpatialMarker(new Vector2d(28 + deltaX, -55), () -> {
//                    bot.intake.setPosition(0.7);
//                    bot.intake.startCollect();
//                })
//                .build();
//
//        toBackdropCycle2[caz] = bot.drive.trajectorySequenceBuilder(toStackCycle2[caz].end())
//                .setReversed(true)
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//                .splineToLinearHeading(new Pose2d(51.5 + deltaX, -36, Math.toRadians(-90)), Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(51.5 + deltaX, 15, Math.toRadians(-90)))
//                .addSpatialMarker(new Vector2d(35, 20), () -> {
//                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
//                    bot.lift.setTarget(1500);
//                    sleep(300);
//                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
//                })
//                .splineToLinearHeading(new Pose2d(36, 45.5, Math.toRadians(-90)), Math.toRadians(90))
//                .build();
//
//        parking[caz] = bot.drive.trajectorySequenceBuilder(toBackdropCycle2[caz].end())
//                .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
//                .addDisplacementMarker(5, () -> {
//                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
//                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
//                    sleep(200);//300
//                    bot.arm.setPosition(0.96);
//                    bot.lift.setTarget(0);
//                })
//                .lineToConstantHeading(new Vector2d(60, 42))
//                .build();
//    }

    public void cycleOne()
    {
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
        bot.intake.setPosition(0.64);
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

        bot.outtake.dropBothPixels();
        sleep(300);
    }

    public void cycleTwo()
    {
        bot.drive.followTrajectorySequenceAsync(toStackCycle2[caz]);

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
        bot.intake.setPosition(0.75);
        sleep(950);

        bot.intake.setPosition(0.3);

        bot.drive.followTrajectorySequenceAsync(toBackdropCycle2[caz]);

        while ((bot.drive.isBusy() || bot.lift.isBusy()) && !isStopRequested()) {
            bot.drive.update();
            bot.lift.update();
            bot.printDebug(telemetry);
            telemetry.addData("er", bot.drive.getLastError());
            telemetry.update();
        }

        bot.outtake.dropBothPixels();
        sleep(300);

        bot.drive.followTrajectorySequenceAsync(parking[caz]);

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

        initDetection();

        pipeline.startDetection(true);

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

        telemetry.addLine("OK. Start");
        telemetry.update();
        telemetry.setAutoClear(false);

        while(!isStarted() && !isStopRequested()){
            telemetry.addData("case", pipeline.getCase());
        }

        waitForStart();
        int temp = pipeline.getCase();
        if(temp == 0){
            caz = 2;
        } else if (temp == -1){
            caz = 1;
        } else if (temp == 1 || temp == -2) {
            caz = 3;
        }

        pipeline.killThis();
        webcam.stopStreaming();


        if (isStopRequested()) {
            return;
        }

        bot.drive.setPoseEstimate(new Pose2d(62, 12, Math.toRadians(0)));
        bot.drive.followTrajectorySequenceAsync(preloads[caz]);

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

        if(caz == 1) {
            cycleOne();

            cycleTwo();
        } else {

            bot.drive.followTrajectorySequenceAsync(bot.drive.trajectorySequenceBuilder(bot.drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(32, 42), Math.toRadians(-90))
                    .addDisplacementMarker(5, () -> {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(200);//300
                        bot.arm.setPosition(0.96);
                        bot.lift.setTarget(0);
                    })
                    .lineToConstantHeading(new Vector2d(60, 42))
                    .build());

            while(!isStopRequested() && (bot.lift.isBusy() || bot.drive.isBusy())){
                bot.lift.update();
                bot.drive.update();
            }

            sleep(2000);
        }
    }

}
