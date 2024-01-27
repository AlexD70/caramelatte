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
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.RedBluePipeline;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "2+P CLOSE RED", group = "auto")
public class AutoParkPlus2CloseRed extends LinearOpMode {
    SampleMecanumDrive rr;
    Intake intake;
    VoltageScaledArm arm;
    Lifter lift;
//    HuskyLensDetection husky;
    ColorSensor sensor;

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

    public PoseStorageV1 redCloseLeftPreload = new PoseStorageV1();
    public PoseStorageV1 redCloseRightPreload = new PoseStorageV1();
    public PoseStorageV1 redCloseCenterPreload = new PoseStorageV1();

    public void buildLeftPreload(){
        redCloseLeftPreload.toSpike =
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, 33, Math.toRadians(-90)), Math.toRadians(90))
                        .build();

        redCloseLeftPreload.toBackdrop =
                rr.trajectorySequenceBuilder(rr.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-10, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1100);
                        })
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

//        HuskyLensDetection.RandomisationCase randomisationCase = HuskyLensDetection.RandomisationCase.UNKNOWN;

//        while(!isStarted()){
//            randomisationCase = husky.getCaseRedClose(telemetry);
//            telemetry.addData("CASE ", randomisationCase);
//            telemetry.update();
//        }

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



        waitForStart();
        sensor.enableLed(false);

//        int randomization = (int) Math.round(Math.random() * 2) - 3;

//        if(randomisationCase != HuskyLensDetection.RandomisationCase.UNKNOWN){
//            randomization = randomisationCase.val;
//        }


        if (randomization == 1) { // STANGA RED
            leftRed();
        } else if (randomization == 0) { // CENTER RED
            centerRed();
        } else { // DREAPTA RED
            rightRed();
        }
    }

    public void runAuto(PoseStorageV1 trajectories) throws InterruptedException {
        rr.followTrajectorySequenceAsync(trajectories.toSpike);

        while(rr.isBusy() && !isStopRequested()){
            rr.update();
        }

        intake.forceAngleServoPos(0.3);
        Thread.sleep(500);
        intake.dropPixel();
        intake.forceAngleServoPos(0.9);
        Thread.sleep(500);

        rr.followTrajectorySequenceAsync(trajectories.toBackdrop);

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

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
        );

        lift.goToPos(Lifter.LifterStates.DOWN);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
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
                        .strafeLeft(19)
                        .build()
        );
    }


    public void leftRed() throws InterruptedException{ //TestCase1Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, 33, Math.toRadians(-90)), Math.toRadians(90))
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
                        .lineToConstantHeading(new Vector2d(-10, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
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

        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
        );

        lift.goToPos(Lifter.LifterStates.DOWN);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
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
                        .strafeLeft(19)
                        .build()
        );
    }

    public void centerRed() throws InterruptedException { //TestCase2Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-25, 28, Math.toRadians(-90)), Math.toRadians(90))
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
                        .lineToConstantHeading(new Vector2d(-12.5, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
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
        rr.followTrajectorySequence(
                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
        );

        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        lift.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
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

    public void rightRed() throws InterruptedException { // TestCase3Red.java
        rr.followTrajectorySequenceAsync(
                rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, 13, Math.toRadians(-90)))
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
                        .lineToConstantHeading(new Vector2d(-18.5, 45.7))
                        .addSpatialMarker(new Vector2d(-12, 45.5), () -> {
                            lift.goToPos(1050);
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
//        rr.followTrajectorySequence(
//                rr.trajectorySequenceBuilder(rr.getPoseEstimate()).forward(2).build()
//        );

        lift.goToPos(Lifter.LifterStates.DOWN);
        arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
        timer.reset();
        while(timer.seconds() < 1 && !isStopRequested()){
            lift.update();
            telemetry.update();
        }
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
}
