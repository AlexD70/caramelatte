package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.RedBluePipeline;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "1 CYCLE CLOSE RED", group = "auto")
public class RedCloseCycle extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    VoltageScaledArm arm;
    Lifter lift;
//    HuskyLensDetection husky;
    ColorSensor sensor;
    TrajectorySequence toSpikeMark, toBackdrop, toStack, toBackdrop2;

    PoseStorageV1 redCloseLeft = new PoseStorageV1();
    PoseStorageV1 redCloseRight = new PoseStorageV1();
    PoseStorageV1 redCloseCenter = new PoseStorageV1();

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

    public void buildRedCloseLeft() {
        redCloseLeft.toSpike =
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, 34, Math.toRadians(-90)), Math.toRadians(90))
                        .build();

        redCloseLeft.toBackdrop =
                rr.trajectorySequenceBuilder(redCloseLeft.toSpike.end())
                        .lineToConstantHeading(new Vector2d(-9, 46.5))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1100);
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                            intake.forceAngleServoPos(0.75);
                        })
                        .build();

        redCloseLeft.toStack = rr.trajectorySequenceBuilder(redCloseLeft.toBackdrop.end())
                .splineToConstantHeading(new Vector2d(6.5, 6), Math.toRadians(-90))
                .lineTo(new Vector2d(6.5, -27))
                .addSpatialMarker(new Vector2d(-12, -42), () -> {
                    intake.forceAngleServoPos(0.3);
                    intake.startCollect();
                })
                .splineToConstantHeading(new Vector2d(-13, -52), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(-13, -52), () -> {
                    lift.keepDown();
                    lift.update();
                })
                .forward(8, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .back(7)
                .forward(7, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .build();

        redCloseLeft.toBackdrop2 =
                rr.trajectorySequenceBuilder(redCloseLeft.toStack.end())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(5, -38),Math.toRadians(90))
                        .addTemporalMarker(0.5, () -> {
                            intake.forceAngleServoPos(0.9);
                            lift.stopKeepDown();
                            lift.goToPos(1);
                        })
                        .lineTo(new Vector2d(5, 6))
                        .splineToConstantHeading(new Vector2d(-9.5, 46.4), Math.toRadians(90))
                        .build();
    }
    public void buildRedCloseRight(){
        redCloseRight.toSpike = rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-18, 13.5, Math.toRadians(-90)))
                .build();

        redCloseRight.toBackdrop = rr.trajectorySequenceBuilder(redCloseRight.toSpike.end())
                .lineToConstantHeading(new Vector2d(-21, 46.5))
                .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                    lift.goToPos(1100);
                    arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                    intake.forceAngleServoPos(0.75);
                })
                .build();

        redCloseRight.toStack = rr.trajectorySequenceBuilder(redCloseRight.toBackdrop.end())
                .splineToConstantHeading(new Vector2d(6.5, 6), Math.toRadians(-90))
                .lineTo(new Vector2d(6.5, -27))
                .addSpatialMarker(new Vector2d(-12, -42), () -> {
                    intake.forceAngleServoPos(0.3);
                    intake.startCollect();
                })
                .splineToConstantHeading(new Vector2d(-14.7, -52), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(-14.7, -52), () -> {
                    lift.keepDown();
                    lift.update();
                })
                .forward(8, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .back(7)
                .forward(7, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .build();

        redCloseRight.toBackdrop2 =
                rr.trajectorySequenceBuilder(redCloseRight.toStack.end())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(6.5, -38), Math.toRadians(90))
                        .addTemporalMarker(0.5, () -> {
                            intake.forceAngleServoPos(0.9);
                            lift.stopKeepDown();
                            lift.goToPos(1);
                        })
                        .lineTo(new Vector2d(6.5, 6))
                        .splineToConstantHeading(new Vector2d(-13, 45.4), Math.toRadians(90))
                        .build();

    }
    public void buildRedCloseCenter() {
        redCloseCenter.toSpike = rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-25, 28, Math.toRadians(-90)), Math.toRadians(90))
                .build();

        redCloseCenter.toBackdrop = rr.trajectorySequenceBuilder(redCloseCenter.toSpike.end())
                .lineToConstantHeading(new Vector2d(-15, 45.8 ))
                .addSpatialMarker(new Vector2d(-15, 45.5), () -> {
                    lift.goToPos(1100);
                    arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                    intake.forceAngleServoPos(0.75);
                })
                .build();

        redCloseCenter.toStack = rr.trajectorySequenceBuilder(redCloseCenter.toBackdrop.end())
                .splineToConstantHeading(new Vector2d(6, 6), Math.toRadians(-90))
                .lineTo(new Vector2d(6, -27))
                .addSpatialMarker(new Vector2d(-12, -42), () -> {
                    intake.forceAngleServoPos(0.3);
                    intake.startCollect();
                })
                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(-12, -52), () -> {
                    lift.keepDown();
                    lift.update();
                })
                .forward(8, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .back(7)
                .forward(7, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(10))
                .build();

        redCloseCenter.toBackdrop2 =
                rr.trajectorySequenceBuilder(redCloseCenter.toStack.end())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(6, -38),Math.toRadians(90))
                        .addTemporalMarker(0.5, () -> {
                            intake.forceAngleServoPos(0.9);
                            lift.stopKeepDown();
                            lift.goToPos(1);
                        })
                        .lineTo(new Vector2d(6, 6))
                        .splineToConstantHeading(new Vector2d(-9.5, 46.3), Math.toRadians(90))
                        .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rr = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new VoltageScaledArm(hardwareMap);
        lift = new Lifter(hardwareMap);
//        husky = new HuskyLensDetection(hardwareMap, "husky");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

        buildRedCloseLeft();
        buildRedCloseRight();
        buildRedCloseCenter();

        initDetection();

        pipeline.startDetection(true);
        int randomization = -2;

        while(opModeInInit()) {
            if (cameraOK) {
                telemetry.addLine("Webcam Ok");
                telemetry.addLine("Ready! Press Play");
                randomization = pipeline.getCase();
                telemetry.addData("case", randomization);
            } else {
                telemetry.addLine("Webcam failed, please RESTART!");
                telemetry.update();
            }
            telemetry.update();
        }

//        HuskyLensDetection.RandomisationCase randomisationCase = HuskyLensDetection.RandomisationCase.UNKNOWN;


//        while(!isStarted()){
//            randomisationCase = husky.getCaseBlueClose(telemetry);
//            telemetry.addData("CASE ", randomisationCase);
//            telemetry.update();
//        }

        waitForStart();
        pipeline.killThis();
        sensor.enableLed(false);

        if(randomization == -2){
            randomization = 1;
        }

//        if(randomisationCase != HuskyLensDetection.RandomisationCase.UNKNOWN){
//            randomization = randomisationCase.val;
//        }

        if (randomization == -1) { // STANGA BLUE
           runAuto(redCloseLeft);
        } else if (randomization == 0) { // CENTER BLUE
            runAuto(redCloseCenter);
        } else { // DREAPTA BLUE
            runAuto(redCloseRight);
        }
    }

    public void runAuto(PoseStorageV1 trajectories) throws InterruptedException {

        rr.followTrajectorySequenceAsync(trajectories.toSpike);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }

        intake.forceAngleServoPos(0.3);
        Thread.sleep(250);
        intake.dropPixel();
        intake.forceAngleServoPos(0.75);
        Thread.sleep(200);

        rr.followTrajectorySequenceAsync(trajectories.toBackdrop);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            arm.printDebug(telemetry);
        }

        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds() < 200 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
        }

        intake.dropPixel();
        //  intake.forceAngleServoPos(0.75);
        timer.reset();
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 0.6 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(trajectories.toStack);
        intake.stopCollect();
        rr.followTrajectorySequenceAsync(trajectories.toBackdrop2);
        while((rr.isBusy() || lift.isBusy()) && !isStopRequested()){
            lift.update();
            rr.update();
        }

//        while(rr.isBusy() && !isStopRequested()){
//            rr.update();
//            lift.update();
//            arm.update(telemetry);
//        }

        intake.forceAngleServoPos(0.75);
        lift.goToPos(Lifter.LifterStates.HIGH);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);

        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        while(lift.isBusy() && !isStopRequested()){
            lift.update(); //  prevent idiot bug
        }

        intake.dropPixel();
        intake.dropPixel();
        sleep(200);
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        while(lift.isBusy() && !isStopRequested()){
            lift.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(20)
                        .build()
        );

    }


    public void rightBluePreload() throws InterruptedException{ //TestCase2.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, -13.5, Math.toRadians(90)))
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

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
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

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public void blueLeft() throws InterruptedException { // TestAxes.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, -33, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }

        intake.forceAngleServoPos(0.3);
        Thread.sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.75);
        Thread.sleep(500);

        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-9, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
                            lift.goToPos(1100);
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                            intake.forceAngleServoPos(0.75);
                        })
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            arm.printDebug(telemetry);
        }

        ElapsedTime timer = new ElapsedTime();
        sleep(200);

        intake.dropPixel();
        //  intake.forceAngleServoPos(0.75);
        timer.reset();
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 0.6 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(6, -6),Math.toRadians(90))
                        .lineTo(new Vector2d(6, 27))
                        .addSpatialMarker(new Vector2d(-12, 42), () -> {
                            intake.forceAngleServoPos(0.3);
                            intake.startCollect();
//                            lift.keepDown();
//                            timer.reset();

                        })
                        .splineToConstantHeading(new Vector2d(-17, 52),Math.toRadians(90))
                        .forward(6, new TranslationalVelocityConstraint(20), new ProfileAccelerationConstraint(10))
                        .back(7)
                        .forward(7, new TranslationalVelocityConstraint(10), new ProfileAccelerationConstraint(10))
                        .build()
        );
//        while(timer.seconds() < 2 && !isStopRequested()){
//            lift.update();
//            arm.update(telemetry);
//            lift.printDebug(telemetry);
//            arm.printDebug(telemetry);
//            telemetry.update();
//        }
        intake.stopCollect();
//        lift.stopKeepDown();

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(5, 38),Math.toRadians(-90))
                        .addTemporalMarker(1, () -> {
                            intake.forceAngleServoPos(0.9);
                        })
                        .lineTo(new Vector2d(5, -6))
                        .splineToConstantHeading(new Vector2d(-9, -45.8), Math.toRadians(-90))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        intake.forceAngleServoPos(0.75);
        lift.goToPos(Lifter.LifterStates.HIGH);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);

        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        intake.dropPixel();
        sleep(200);
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
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

    // start trajectories



    public void blueLeftRR() throws InterruptedException { // TestAxes.java

        rr.followTrajectorySequenceAsync(toSpikeMark);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }

        intake.forceAngleServoPos(0.3);
        Thread.sleep(250);
        intake.dropPixel();
        intake.forceAngleServoPos(0.75);
        Thread.sleep(200);

        rr.followTrajectorySequenceAsync(toBackdrop);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            arm.printDebug(telemetry);
        }

        ElapsedTime timer = new ElapsedTime();
        sleep(200);

        intake.dropPixel();
        //  intake.forceAngleServoPos(0.75);
        timer.reset();
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 0.6 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

        rr.followTrajectorySequence(toStack);
//        while(timer.seconds() < 2 && !isStopRequested()){
//            lift.update();
//            arm.update(telemetry);
//            lift.printDebug(telemetry);
//            arm.printDebug(telemetry);
//            telemetry.update();
//        }
        intake.stopCollect();
//        lift.stopKeepDown();

        rr.followTrajectorySequence(toBackdrop2);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        intake.forceAngleServoPos(0.75);
        lift.goToPos(Lifter.LifterStates.HIGH);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);

        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        while(arm.isArmBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
        }

        intake.dropPixel();
        sleep(200);
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
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

    public void centerBlue() throws InterruptedException{ //TestCase2.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-25, -28, Math.toRadians(90)), Math.toRadians(-90))
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
                        .lineToConstantHeading(new Vector2d(-15, -46.1))
                        .addSpatialMarker(new Vector2d(-15, -45.5), () -> {
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
        while(timer.seconds() < 1.5 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

//        rr.followTrajectorySequence(
//                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
//                        .strafeRight(22)
//                        .build()
//        );

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(6,-6),Math.toRadians(90))
                        .lineTo(new Vector2d(6,25))
                        .addSpatialMarker(new Vector2d(-12, 42), () -> {
                            intake.forceAngleServoPos(0.3);
                            intake.startCollect();
//                            timer.reset();

                        })
                        .splineToConstantHeading(new Vector2d(-16, 55), Math.toRadians(90))
                        .forward(5,
                                new MecanumVelocityConstraint(20, 20),
                                new ProfileAccelerationConstraint(10))
                        .back(6.5)
                        .forward(6.9)
                        .build()
        );
//        while(timer.seconds() < 2 && !isStopRequested()){
//            lift.update();
//            arm.update(telemetry);
//            lift.printDebug(telemetry);
//            arm.printDebug(telemetry);
//            telemetry.update();
//        }
        intake.forceAngleServoPos(0.9);
        sleep(200);
        intake.stopCollect();

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(6, 38),Math.toRadians(-90))
                        .lineTo(new Vector2d(6,-6))
                        .splineToConstantHeading(new Vector2d(-8, -46),Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-5, -30), () -> {
                            intake.forceAngleServoPos(0.75);
                            lift.goToPos(1400);
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                        })
                        .build()
        );
        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }

        while(lift.isBusy() && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
        }

        intake.dropPixel();
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(arm.isArmBusy() && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            telemetry.update();
        }

    }

    public void rightBlue() throws InterruptedException{ //TestCase2.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(90)))
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
                        .lineToConstantHeading(new Vector2d(-20, -45.4))
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
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }

        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }

//        rr.followTrajectorySequence(
//                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
//                        .strafeRight(26)
//                        .build()
//        );
        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(-40,-6),Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(-40, 42), () -> {
                            intake.forceAngleServoPos(0.3);
                            intake.startCollect();
//                            timer.reset();

                        })
                        .lineTo(new Vector2d(-44.5,55))
                        .lineTo(new Vector2d(-44.5,62),//60
                                new TranslationalVelocityConstraint(30),
                                new ProfileAccelerationConstraint(15))//10
//                        .splineToConstantHeading(new Vector2d(-25,60),Math.toRadians(90),
//                                new TranslationalVelocityConstraint(30),
//                                new ProfileAccelerationConstraint(10))
                        .back(6.5)
                        .forward(6.9)
                        .build()
        );
//        while(timer.seconds() < 2 && !isStopRequested()){
//            lift.update();
//            arm.update(telemetry);
//            lift.printDebug(telemetry);
//            arm.printDebug(telemetry);
//            telemetry.update();
//        }
        intake.stopCollect();

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .setReversed(true)
//                        .splineToConstantHeading(new Vector2d(-25,38),Math.toRadians(-90))
                        .lineTo(new Vector2d(-40,-6))
                        .splineToConstantHeading(new Vector2d(-9,-46.8),Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-5, -30), () -> {
                            intake.forceAngleServoPos(0.75);
                            lift.goToPos(1400);
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
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            lift.update();
            lift.printDebug(telemetry);
            telemetry.update();
        }
        intake.dropPixel();
        sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            arm.update(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1.5 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
            lift.printDebug(telemetry);
            arm.printDebug(telemetry);
            telemetry.update();
        }


    }
}
