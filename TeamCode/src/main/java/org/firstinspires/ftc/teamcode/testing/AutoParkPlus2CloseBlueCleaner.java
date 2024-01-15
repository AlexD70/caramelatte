package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.HuskyLensDetection;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotAutoActions;


@Autonomous(name = "2+P close blue untested", group = "cleanup on autos")
public class AutoParkPlus2CloseBlueCleaner extends LinearOpMode {
    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        bot.setTelemetry(telemetry);
        bot.initAuto(hardwareMap, this, RobotAutoActions.Side.BLUE_CLOSE);

        HuskyLensDetection.RandomisationCase randomisationCase = RobotAutoActions.getRandomisation();

        waitForStart();

        int randomization;
        if(randomisationCase == HuskyLensDetection.RandomisationCase.UNKNOWN){
            randomization = (int) Math.round(Math.random() * 2) - 3;
        } else {
            randomization = randomisationCase.val;
        }

        if (randomization == -1) { // STANGA BLUE
            blueLeft();
        } else if (randomization == 0) { // CENTER BLUE
            centerBlue();
        } else { // DREAPTA BLUE
            rightBlue();
        }
    }


    public void blueLeft() throws InterruptedException{ // TestAxes.java

        RobotAutoActions.rrFollow( // TO SPIKE MARK
                RobotAutoActions.getBuilder()
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-20, -33, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        RobotAutoActions.dropPixelOnSpikeMark();

        RobotAutoActions.rrFollowInParallelWith( // TO BACKDROP
                RobotAutoActions.fromPoseEstimate()
                        .lineToConstantHeading(new Vector2d(-9, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
                            bot.lifter.goToPos(1100);
                        })
                        .build(),

                RobotAutoActions.updateAllLambda,
                RobotAutoActions.rrBusyBoolSupplier
        );

        while(bot.lifter.isBusy() && !isStopRequested()){
            bot.lifter.update();
        }

        RobotAutoActions.placePixelsOnBackdrop(3, false);

        RobotAutoActions.endAutonomy(1, 3);

        RobotAutoActions.rrFollow( // PARK LEFT
                RobotAutoActions.fromPoseEstimate()
                        .strafeRight(20)
                        .build()
        );
    }

    public void centerBlue() throws InterruptedException{ //TestCase2.java

        RobotAutoActions.rrFollow( // TO SPIKE MARK
                RobotAutoActions.getBuilder()
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-25, -28, Math.toRadians(90)), Math.toRadians(-90))
                        .build()
        );

        RobotAutoActions.dropPixelOnSpikeMark();

        RobotAutoActions.rrFollowInParallelWith( // TO BACKDROP
                RobotAutoActions.fromPoseEstimate()
                        .lineToConstantHeading(new Vector2d(-12, -46.1))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
                            bot.lifter.goToPos(1100);
                        })
                        .build(),
                RobotAutoActions.updateAllLambda,
                RobotAutoActions.rrBusyBoolSupplier
        );

        while(bot.lifter.isBusy() && !isStopRequested()){
            bot.lifter.update();
        }

        RobotAutoActions.placePixelsOnBackdrop(3, false);

        RobotAutoActions.endAutonomy(1, 3);

        RobotAutoActions.rrFollow( // PARK LEFT
                RobotAutoActions.fromPoseEstimate()
                        .strafeRight(22)
                        .build()
        );

    }

    public void rightBlue() throws InterruptedException{ //TestCase2.java
        RobotAutoActions.rrFollow(
                RobotAutoActions.getBuilder()
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20, -13.5, Math.toRadians(90)))
                        .build()
        );

        RobotAutoActions.dropPixelOnSpikeMark();

        RobotAutoActions.rrFollowInParallelWith(
                RobotAutoActions.fromPoseEstimate()
                        .lineToConstantHeading(new Vector2d(-18, -45.4))
                        .addSpatialMarker(new Vector2d(-12, -45.5), () -> {
                            bot.lifter.goToPos(1100);
                        })
                        .build(),
                RobotAutoActions.updateAllLambda,
                RobotAutoActions.rrBusyBoolSupplier
        );

        while(bot.lifter.isBusy() && !isStopRequested()){
            bot.lifter.update();
        }

        RobotAutoActions.placePixelsOnBackdrop(3, false);
        RobotAutoActions.endAutonomy(1, 3);

        RobotAutoActions.rrFollow(
                RobotAutoActions.fromPoseEstimate()
                        .strafeRight(26)
                        .build()
        );

    }
}
