package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.BluePipeline;
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "2+P FAR BLUE", group = "auto")
public class AutoParkPlus2FarBlue extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    VoltageScaledArm arm;
    Lifter lift;
    HuskyLensDetection husky;
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
//        husky = new HuskyLensDetection(hardwareMap, "husky");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

//        HuskyLensDetection.RandomisationCase randomisationCase = HuskyLensDetection.RandomisationCase.UNKNOWN;

        initDetection();

        pipeline.startDetection(false);
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

        if(randomization == -2){
            randomization = 1;
        }

        waitForStart();
        sensor.enableLed(false);

        sleep(2000);
        if (randomization == 1) { // STANGA BLUE
            blueLeft();
        } else if (randomization == 0) { // CENTER BLUE
            centerBlue();
        } else { // DREAPTA BLUE
            rightBlue();
        }

    }

    public void blueLeft() throws InterruptedException{ // TestCase1FarBlue.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-15, -14.8, Math.toRadians(90)), Math.toRadians(-90))
                        .back(25)
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
//        sleep(3000);
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .back(20)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(0, -68), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-8.8, -100), Math.toRadians(-90))
                        .addSpatialMarker(new Vector2d(-3, -90), () -> {
                            lift.goToPos(1100);
                        })
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
        }


        ElapsedTime lifterTimeout = new ElapsedTime();
        lifterTimeout.reset();
        while(lift.isBusy() && lifterTimeout.seconds() < 0.6 && !isStopRequested()){
            lift.update();
        }

        intake.forceAngleServoPos(0.75);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
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
//                        .strafeLeft(30)
//                        .build()
//        );

    }

    public void centerBlue() throws InterruptedException{ //TestCase2FarBlue.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .back(52)
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
//        sleep(3000);
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(-52, 0, 0))
                        .forward(3)
                        .addSpatialMarker(new Vector2d(-30, -70), () -> {
                            lift.goToPos(1100);
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                            intake.forceAngleServoPos(0.75);
                        })
                        .lineToLinearHeading(new Pose2d(-43, -60, Math.toRadians(90)))
                        .back(10)
                        //.strafeRight(63, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(20))
                        .setTangent(Math.toRadians(-90))
                        .lineToLinearHeading(new Pose2d(-17.3, -101, Math.toRadians(90)))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }


        ElapsedTime lifterTimeout = new ElapsedTime();
        lifterTimeout.reset();
        while(arm.isArmBusy() && lifterTimeout.seconds() < 0.6 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
        }

        //intake.forceAngleServoPos(0.7);
        //arm.setArmTarget(Arm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 0.5 && !isStopRequested()){
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

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(25)
                        .build()
        );
    }

    public void rightBlue() throws InterruptedException{ //TestCase3FarBlue.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-45, -6, Math.toRadians(30)))
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
//        sleep(3000);
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .addSpatialMarker(new Vector2d(-30, -70), () -> {
                            lift.goToPos(1100);
                            arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
                            intake.forceAngleServoPos(0.75);
                        })
                        .lineToLinearHeading(new Pose2d(-41, -60, Math.toRadians(90)))
                        .back(10)
                        //.strafeRight(63, new TranslationalVelocityConstraint(30), new ProfileAccelerationConstraint(20))
                        .lineToLinearHeading(new Pose2d(-25, -100.3, Math.toRadians(90)))
                        .build()
        );

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
            lift.update();
            arm.update(telemetry);
        }


        ElapsedTime lifterTimeout = new ElapsedTime();
        lifterTimeout.reset();
        while(arm.isArmBusy() && lifterTimeout.seconds() < 0.6 && !isStopRequested()){
            lift.update();
            arm.update(telemetry);
        }

        arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 2 && !isStopRequested()){
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

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .strafeLeft(20)
                        .build()
        );
    }
}
