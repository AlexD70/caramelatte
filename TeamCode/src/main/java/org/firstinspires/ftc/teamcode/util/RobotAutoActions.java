package org.firstinspires.ftc.teamcode.util;

import androidx.core.util.Supplier;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class RobotAutoActions {
    public static Robot bot;
    public static ElapsedTime timer = new ElapsedTime();
    public static LinearOpMode opmode;
    public static Pose2d lastKnownPose = new Pose2d(0, 0, 0);

    public enum Side {
        RED_CLOSE, RED_FAR, BLUE_CLOSE, BLUE_FAR
    }
    public static Side currentSide;

    public static void initAutoActions(Side side, LinearOpMode _opmode){
        bot = Robot.getRobotInstance(null);
        currentSide = side;
        opmode = _opmode;
    }

    public static void initAutoActions(Robot robot, Side side, LinearOpMode _opmode){
        bot = robot;
        currentSide = side;
        opmode = _opmode;
    }

    public static HuskyLensDetection.RandomisationCase getRandomisation() throws InterruptedException {
        HuskyLensDetection.RandomisationCase randCase = HuskyLensDetection.RandomisationCase.UNKNOWN;

        while(!opmode.isStopRequested() && !opmode.isStarted()){
            if(currentSide == Side.BLUE_CLOSE){
//                randCase = bot.husky.getCaseBlueClose(bot.telemetry);
            } else if (currentSide == Side.BLUE_FAR){
//                randCase = bot.husky.getCaseBlueFar(bot.telemetry);
            } else if (currentSide == Side.RED_CLOSE){
//                randCase = bot.husky.getCaseRedClose(bot.telemetry);
            } else {
//                randCase = bot.husky.getCaseRedFar(bot.telemetry);
            }

        }

        return randCase;
    }

    public static TrajectorySequenceBuilder getBuilder(){
        return bot.rr.trajectorySequenceBuilder(lastKnownPose);
    }

    public static TrajectorySequenceBuilder getBuilderForcePose(Pose2d pose){
        lastKnownPose = pose;
        return bot.rr.trajectorySequenceBuilder(pose);
    }

    public static TrajectorySequenceBuilder fromPoseEstimate(){
        return bot.rr.trajectorySequenceBuilder(bot.rr.getPoseEstimate());
    }

    public static void rrFollow(TrajectorySequence seq){
        bot.rr.followTrajectorySequence(seq);
        lastKnownPose = seq.end();
    }

    public static void rrFollowAsync(TrajectorySequence seq){
        bot.rr.followTrajectorySequenceAsync(seq);
        lastKnownPose = seq.end();
    }

    public static void rrFollowInParallelWith(TrajectorySequence seq, Runnable r, Supplier<Boolean> continueCondition){
        bot.rr.followTrajectorySequenceAsync(seq);
        while(continueCondition.get()){
            bot.rr.update();
            r.run();
        }
        lastKnownPose = seq.end();
    }

    public static void dropPixelOnSpikeMark() throws InterruptedException{
        bot.intake.forceAngleServoPos(0.5);
        opmode.sleep(500);

        bot.intake.dropPixel();
        bot.intake.forceAngleServoPos(0.9);
        opmode.sleep(500);
    }

    public static void placePixelsOnBackdrop(double timeout, boolean twoPixels) throws InterruptedException{
        bot.intake.forceAngleServoPos(0.75);
        bot.arm.setArmTarget(Arm.ArmPositions.PLACE);
        timer.reset();
        while(timer.seconds() < timeout && opmode.opModeIsActive()){
            updateAllLambda.run();
        }

        if(twoPixels) {
            bot.intake.dropPixel();
        }
        bot.intake.dropPixel();
    }

    public static void endAutonomy(double timeout1, double timeout2){
        bot.intake.forceAngleServoPos(0.9);
        bot.arm.forceArmToPosition(0);

        timer.reset();
        while(timer.seconds() < timeout1 && opmode.opModeIsActive()){
            updateAllLambda.run();
        }

        bot.lifter.goToPos(Lifter.LifterStates.DOWN);
        timer.reset();
        while(timer.seconds() < timeout2 && opmode.opModeIsActive()){
            updateAllLambda.run();
        }
    }

    public static Runnable updateAllLambda = () -> MiscActions.bulkUpdate(bot, bot.rr, bot.telemetry);
    public static Supplier<Boolean> rrBusyBoolSupplier = () -> bot.rr.isBusy() && opmode.opModeIsActive();
}
