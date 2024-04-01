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
public class TeleOpV1 extends LinearOpMode {
    Robot bot = new Robot();
    Controller ctrl1, ctrl2;

    public double weight = 1;
    public boolean isInPlaceMode = false;
    public boolean pixelsLocked = false;

    public boolean isLimited = true;
    public static double LIFTER_MANUAL_WEIGHT = 14, LIFTER_MANUAL_THRESH = 0.2;
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
                bot.lift.setTarget(Range.clip((int) (lifterInitial + deltaLifter), 0, 2400));
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

        MiscActions.bulkSetThreshes(ctrl1, ctrl2, 0.4);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
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
                bot.outtake.catchPixels();
                bot.arm.setPosition(Arm.ArmPositions.PLACE);
                if(!isInPlaceMode) {
                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
                    bot.lift.setTarget(2000);
                    isInPlaceMode = true;
                    sleep(300);
                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                }
            }

            // lifter high when in place
            if(ctrl2.dpadUp.isPressed()){
                if(!isInPlaceMode) {
                    bot.outtake.gearToPos(Outtake.GearStates.PLACE);
                    sleep(300);
                    bot.arm.setPosition(Arm.ArmPositions.PLACE);
                    isInPlaceMode = true;
                }
                bot.lift.setTarget(2400);
            }

            // to collect state
            if(ctrl2.circle.isPressed()){
                if(isInPlaceMode) {
                    bot.outtake.rotateToAngle(Outtake.BoxRotationStates.COLLECT_POS);
                    bot.outtake.gearToPos(Outtake.GearStates.COLLECT);
                    sleep(300);
                    bot.arm.setPosition(Arm.ArmPositions.COLLECT);
                    isInPlaceMode = false;
                    bot.outtake.dropBothPixels();
                    pixelsLocked = false;
                }
                bot.lift.setTarget(0);
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
                bot.lift.setTarget(600);
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
