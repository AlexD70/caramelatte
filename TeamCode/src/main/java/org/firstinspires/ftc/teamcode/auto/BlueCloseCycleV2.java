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
import org.firstinspires.ftc.teamcode.util.BluePipeline;
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "1 CYCLE CLOSE BLUE V2", group = "auto")
public class BlueCloseCycleV2 extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    VoltageScaledArm arm;
    Lifter lift;
    ColorSensor sensor;
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

    @Override
    public void runOpMode() throws InterruptedException {
        rr = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        arm = new VoltageScaledArm(hardwareMap);
        lift = new Lifter(hardwareMap);
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

        initDetection();

        pipeline.startDetection(true);

        if (cameraOK) {
            telemetry.addLine("Webcam Ok");

            telemetry.addLine("Ready! Press Play");
        } else {
            telemetry.addLine("Webcam failed, please RESTART!");
            telemetry.update();
            sleep(1000);
        }

        while(opModeInInit()){
            telemetry.addData("case", pipeline.whichCase);
        }

        waitForStart();
        sensor.enableLed(false);

        pipeline.killThis();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        int randomization = (int) Math.round(Math.random() * 2) - 3;

        if(pipeline.whichCase != -2){
            randomization = pipeline.whichCase;
        }

        if (randomization == 1) { // STANGA BLUE
            blueLeft();
        } else if (randomization == 0) { // CENTER BLUE
            centerBlue();
        } else { // DREAPTA BLUE
            rightBluePreload();
        }
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
                        .addDisplacementMarker(3, () -> {
                            intake.forceAngleServoPos(0.9);
                        })
                        .lineTo(new Vector2d(5, -6))
                        .splineToConstantHeading(new Vector2d(-9, -45.8), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-5, -30), () -> {
                            lift.goToPos(1300);
                        })
                        .addSpatialMarker(new Vector2d(-7, -40), () -> {
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                        })
                        .build()
        );
        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }
        intake.forceAngleServoPos(0.75);

        while(lift.isBusy() && !isStopRequested()){
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
