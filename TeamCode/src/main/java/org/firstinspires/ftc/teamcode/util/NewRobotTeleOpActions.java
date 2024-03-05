package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Controller;

public class NewRobotTeleOpActions {
    public static Robot bot;
    public static ElapsedTime timer = new ElapsedTime();
    public static boolean isInHangingMode = false;
    public static boolean isLimited = true;

    public static void initActions(){
        bot = Robot.getRobotInstance(null);
    }

    public static void initActions(Robot robot){
        bot = robot;
    }

    public static void start(){
        timer.reset();
        bot.setState(Robot.RobotTeleOpStates.RUNNING);
        isInHangingMode = false;
    }

    public static void drive(@NonNull Controller ctrl, double weight){
        bot.drive.setDrivePowerWeighted(new Pose2d(-ctrl.rightStickX, -ctrl.leftStickX, ctrl.leftStickY), weight);
    }

    public static void drive2(@NonNull Controller ctrl, double weight){
        bot.drive2.setDrivePower(new Pose2d(ctrl.leftStickY, ctrl.leftStickX * 1.3, -ctrl.rightStickX).times(weight));
    }

    public static void toCollectState(boolean button){
        if (button) {
            bot.lifter.goToPos(Lifter.LifterStates.DOWN);
            bot.left_armservo.setArmTarget(NEWArm.NewArmPositions.COLLECT);
            bot.right_armservo.setArmTarget(NEWArm.NewArmPositions.COLLECT);
            bot.s_angleAdjust.pixelToAngle(Outtake.AngleAdjustStates.INIT);
            bot.s_outtakeAngleAdjust.outtakeToAngle(Outtake.OUTTAKE_AngleAdjustStates.COLLECT_POS);
            bot.s_angleAdjust_broom.broomToAngle(NewIntake.AngleAdjustStates_BROOM.COLLECT_POS);
//            bot.broom.startCollect();
        }
    }

    public static void toPlaceState(boolean button){
        if(button){
            bot.lifter.goToPos(Lifter.LifterStates.MID.pos);
            bot.left_armservo.setArmTarget(NEWArm.NewArmPositions.PLACE);
            bot.right_armservo.setArmTarget(NEWArm.NewArmPositions.PLACE_AUTO);
            bot.s_outtakeAngleAdjust.outtakeToAngle(Outtake.OUTTAKE_AngleAdjustStates.PLACE);
        }
    }

    private static int servoDirection = 0;
    public static void controlIntakeManually(Button collect, Button drop){
        if(collect.isPressed()){
            bot.broom.startCollect();
            servoDirection = 1;
        } else if (drop.isPressed()){
            bot.broom.startEject();
            servoDirection = -1;
        }

        if(collect.isReleased() && servoDirection == 1){
            bot.broom.stopCollect();
        } else if (drop.isReleased() && servoDirection == -1){
            bot.broom.stopCollect();
        }
    }

    public static double LIFTER_MANUAL_WEIGHT = 12, LIFTER_MANUAL_THRESH = 0.2;
    public static double deltaLifter = 0;
    public static void controlLifterManually(double movement){
        if(bot.lifter.isBusy()){
            deltaLifter = 0;
            return;
        }

        if(Math.abs(movement) > LIFTER_MANUAL_THRESH) {
            deltaLifter += movement * LIFTER_MANUAL_WEIGHT;
            bot.lifter.goToPos(Range.clip((int) (deltaLifter) + bot.lifter.getPos(), 0, 2200));
        }
    }

    public static void overrideLimits(boolean button){
        if(button){
            isLimited = false;
        }
    }

    public static int lifterInManual = 0, lifterInitial = 0;
    public static void controlLifterManually2(double movement){
        if(bot.lifter.isBusy()){
            deltaLifter = 0;
            lifterInManual = 0;
            return;
        }

        if(Math.abs(movement) > LIFTER_MANUAL_THRESH){
            lifterInManual += 1;
            if(lifterInManual == 1){
                lifterInitial = bot.lifter.getPos();
            }
            deltaLifter += movement * LIFTER_MANUAL_WEIGHT;
            if(isLimited) {
                bot.lifter.goToPos(Range.clip((int) (lifterInitial + deltaLifter), 0, 2200));
            } else {
                bot.lifter.goToPos((int)(lifterInitial + deltaLifter));
            }
        } else {
            lifterInManual = 0;
            deltaLifter = 0;
        }
    }

    public static double armInManual = 0, armInitial = 0;
    public static void controlArmManually2(double movement){
        if(bot.arm.isArmBusy()){
            armInitial = 0;
            armInManual = 0;
            return;
        }

        if(Math.abs(movement) > ARM_MANUAL_THRESH){
            armInManual += 1;
            if(armInManual == 1){
                armInitial = bot.left_armservo.getArmPosition();
            }
            deltaArm += movement * ARM_MANUAL_WEIGHT;
            if (isLimited) {
                bot.left_armservo.setArmTarget(Range.clip((int) (armInitial + deltaArm), -10, 1900));
                bot.right_armservo.setArmTarget(Range.clip((int) (armInitial + deltaArm), -10, 1900));

            } else {
                bot.left_armservo.setArmTarget((int)(armInitial + deltaArm));
                bot.right_armservo.setArmTarget((int)(armInitial + deltaArm));

            }
        } else {
            armInManual = 0;
            deltaArm = 0;
        }
    }

    public static double ARM_MANUAL_WEIGHT = 7, ARM_MANUAL_THRESH = 0.2;
    public static double deltaArm = 0;
    public static void controlArmManually(double movement){
        if(bot.arm.isArmBusy() || Math.abs(movement) < ARM_MANUAL_THRESH){
            deltaArm = 0;
            return;
        }

        if(Math.abs(movement) > ARM_MANUAL_THRESH){
            deltaArm += movement * ARM_MANUAL_WEIGHT;
            bot.left_armservo.setArmTarget(bot.left_armservo.getArmPosition() + (int)(deltaArm));
        }
    }

    public static void controlLifter(Controller ctrl){
        if(ctrl.dpadUp.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.DOWN);
        } else if (ctrl.dpadDown.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.HIGH);
        } else if (ctrl.dpadLeft.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.ULTRA_HIGH);
        }
    }

    public static void hangingModeInit(boolean button){
        if(timer.seconds() < 3){
            return;
        }

        if(button){
            //bot.setState(Robot.RobotTeleOpStates.HANGING_MODE);
            isInHangingMode = true;
            bot.left_armservo.setArmTarget(NEWArm.NewArmPositions.HANG);
            bot.right_armservo.setArmTarget(NEWArm.NewArmPositions.HANG);
            bot.lifter.goToPos(Lifter.LifterStates.MID);
        }
    }

    public static double HANG_TIME_MIN = 2.5;
    public static void tryHang(boolean button){
        if(bot.getTeleOpState() != Robot.RobotTeleOpStates.HANGING_MODE){
            return;
        }

        double time = timer.seconds();
        if(button){
            bot.lifter.goToPos(Lifter.LifterStates.HANG);
            while(bot.lifter.isBusy()){
                bot.lifter.update();
            }
            if(Math.abs(timer.seconds() - time) < HANG_TIME_MIN){
                bot.lifter.goToPos(Lifter.LifterStates.MID);
            } else {
                bot.isHanged = true;
            }
        }
    }

    public static void killWheels(){
        for(DcMotorEx m : bot.drive.motors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public static void cancelHanging(boolean button){
        if(bot.isHanged){
            bot.lifter.goToPos(Lifter.LifterStates.MID);
            while(bot.lifter.isBusy()){
                bot.lifter.update();
            }
            bot.isHanged = false;
        } else if (bot.getTeleOpState() == Robot.RobotTeleOpStates.HANGING_MODE){
            bot.setState(Robot.RobotTeleOpStates.RUNNING);
            isInHangingMode = false;
        }
    }

    public static double LIFTER_HANGING_WEIGHT = 30, LIFTER_HANGING_THRESH = 0.5;
    public static double deltaLiftHanging = 0;
    public static void lifterHangingOverride(double movement){
        if(movement == 0){
            deltaLiftHanging = 0;
        }

        if(bot.isHanged && Math.abs(movement) > LIFTER_HANGING_THRESH){
            deltaLiftHanging = movement * LIFTER_HANGING_WEIGHT;
            bot.lifter.goToPos(bot.lifter.getPos() + (int)(deltaLiftHanging));
        }
    }

    public static void launchPlane(boolean button){
        if(timer.seconds() < 3){
            return;
        }

        if(button){
            bot.launcher.launchPlane();
        }
    }

    public static Pose2d PLANE_LAUNCH = new Pose2d(0, -19, 1.5);
    public static void driveToPlaneLaunchZone(){
        return;
    }
}
