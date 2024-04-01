package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MiscActions;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
public class TeleOpV2 extends LinearOpMode {
    Robot bot = new Robot();
    Controller ctrl1, ctrl2;

    public double weight = 1;
    public boolean isInPlaceMode = false;
    public boolean isCollecting = false;
    public boolean pixelsLocked = false;

    public boolean isLimited = true;
    public static double LIFTER_MANUAL_WEIGHT = 16, LIFTER_MANUAL_THRESH = 0.2;
    public void overrideLimits(boolean button){
        if(button){
            isLimited = false;
        }
    }

    public int lifterInManual = 0, lifterInitial = 0, deltaLifter;
    public double deltaRotate;

    public Thread uniqueThread = new Thread();

    public void controlLifterManually(double movement){
        if(bot.lift.isBusy()){
            deltaLifter = 0;
            lifterInManual = 0;
            return;
        }

        if(Math.abs(movement) > LIFTER_MANUAL_THRESH){
            lifterInManual += 1;
            if(lifterInManual == 1){
                lifterInitial = bot.lift.getPosition();
            }
            deltaLifter += movement * LIFTER_MANUAL_WEIGHT;
            if(isLimited) {
                bot.lift.setTarget(Range.clip((int) (lifterInitial + deltaLifter), 0, 2800));
            } else {
                bot.lift.setTarget((int)(lifterInitial + deltaLifter));
            }
        } else {
            lifterInManual = 0;
            deltaLifter = 0;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        ctrl1 = new Controller(gamepad1);
        ctrl2 = new Controller(gamepad2);

        bot.outtake.dropBothPixels();

        MiscActions.bulkSetThreshes(ctrl1, ctrl2, 0.4);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(bot.intake.getPosition() != -1 && isInPlaceMode){
                bot.intake.setPosition(Intake.BroomStates.NEUTRAL);
                bot.intake.stopCollect();
            }

            bot.printDebug(telemetry);
            MiscActions.bulkUpdate(ctrl1, ctrl2, bot, telemetry);

            // drive
            bot.drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * 1.4,
                    -gamepad1.right_stick_x
            ).times(weight));

            // to place state
            if(ctrl2.cross.isPressed()){
                uniqueThread.interrupt();
                uniqueThread = new Thread(() -> {
                    if(!isInPlaceMode) {
                        bot.outtake.catchPixels();
                        pixelsLocked = true;
                        sleep(300);
                        if (Thread.currentThread().isInterrupted()) {
                            return;
                        }
                        bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        bot.lift.setTarget(2000);
                        sleep(300);
                        if (Thread.currentThread().isInterrupted()) {
                            return;
                        }
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        isInPlaceMode = true;
                    }
                });
                uniqueThread.start();
            }

            // lifter high when in place
            if(ctrl2.dpadUp.isPressed()){
                uniqueThread.interrupt();
                uniqueThread = new Thread(() -> {
                    if (!isInPlaceMode) {
                        bot.outtake.catchPixels();
                        pixelsLocked = true;
                        sleep(300);
                        if(Thread.currentThread().isInterrupted()){
                            return;
                        }
                        bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                        uniqueThread = new Thread(() -> {
                            sleep(300);
                            if(Thread.currentThread().isInterrupted()){
                                return;
                            }
                            bot.arm.setPosition(Arm.ArmPositions.PLACE);
                        });
                        isInPlaceMode = true;
                    }
                    bot.lift.setTarget(2500);
                });

                uniqueThread.start();
            }

            // to collect state
            if(ctrl2.circle.isPressed()){
                uniqueThread.interrupt();
                uniqueThread = new Thread(() -> {
                    if (isInPlaceMode) {
                        bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                        bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                        sleep(300);
                        if(Thread.currentThread().isInterrupted()){
                            return;
                        }
                        bot.lift.setTarget(0);
                        bot.arm.setPosition(Arm.ArmPositions.COLLECT);
                        bot.outtake.dropBothPixels();
                        pixelsLocked = false;
                        isInPlaceMode = false;
                    }
                });
                uniqueThread.start();
            }


            if(ctrl2.isRightTriggerPressed()){
                bot.intake.setPosition(Intake.BroomStates.COLLECT_POS);
                bot.intake.startCollect();
            }

            if (ctrl2.square.isPressed()){
                bot.intake.stopCollect();
                bot.intake.setPosition(Intake.BroomStates.INIT);
            }

            if (ctrl2.isLeftTriggerPressed()){
                bot.intake.startEject();
            }

            if (ctrl2.bumperRight.isPressed()){
                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.LEFT);
            }

            if (ctrl2.bumperLeft.isPressed()){
                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.RIGHT);
            }

            if (ctrl2.triangle.isPressed()){
                bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
            }
                // pixel control

            // drop left & right
            if(ctrl1.dpadLeft.isPressed()){
                bot.outtake.dropLeftPixel();
                pixelsLocked = false;
            }

            if(ctrl1.dpadRight.isPressed()){
                bot.outtake.dropRightPixel();
                pixelsLocked = false;
            }

            // lock/unlock pixels
            if(ctrl1.cross.isPressed()){
                if(pixelsLocked){
                    bot.outtake.dropBothPixels();
                    pixelsLocked = false;
                } else {
                    bot.outtake.catchPixels();
                    pixelsLocked = true;
                }
            }

            // plane control
            if(ctrl1.bumperLeft.isPressed()){
                bot.launcher.launchPlane();
            }

            // hanging bot
            if(ctrl1.isLeftTriggerPressed()){
                if(isInPlaceMode) {
                    bot.lift.setTarget(600);
                }
            }

            if(ctrl1.bumperRight.isPressed()){
                weight = 0.5;
            } else if (ctrl1.bumperRight.isReleased()){
                weight = 1;
            }

            // lift manual
            controlLifterManually(-ctrl2.rightStickY);

            // limit override
            overrideLimits(ctrl2.share.isPressed());
        }
    }
}
