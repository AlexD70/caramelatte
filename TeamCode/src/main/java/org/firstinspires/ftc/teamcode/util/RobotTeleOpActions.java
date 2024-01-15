package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Controller;

public class RobotTeleOpActions {
    public static Robot bot;
    public static ElapsedTime timer = new ElapsedTime();
    public static boolean isInHangingMode = false;

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
            bot.arm.setArmTarget(Arm.ArmPositions.COLLECT);
            bot.intake.toAngle(Intake.AngleAdjustStates.COLLECT_POS);
        }
    }

    public static void toPlaceState(boolean button){
        if(button){
            bot.arm.setArmTarget(Arm.ArmPositions.PLACE);
            bot.intake.forceAngleServoPos(0.75);
        }
    }

    private static int servoDirection = 0;
    public static void controlIntakeManually(Button collect, Button drop){
        if(collect.isPressed()){
            bot.intake.startCollect();
            servoDirection = 1;
        } else if (drop.isPressed()){
            bot.intake.startEject();
            servoDirection = -1;
        }

        if(collect.isReleased() && servoDirection == 1){
            bot.intake.stopCollect();
        } else if (drop.isReleased() && servoDirection == -1){
            bot.intake.stopCollect();
        }
    }

    public static void controlIntakeManually(Button collect, Button drop, boolean fast){
        if(collect.isPressed()){
            if(fast) {
                bot.intake.startCollectFast();
            } else {
                bot.intake.startCollect();
            }
            servoDirection = 1;
        } else if (drop.isPressed()){
            if(fast){
                bot.intake.startEjectFast();
            } else {
                bot.intake.startEject();
            }
            servoDirection = -1;
        }

        if(collect.isReleased() && servoDirection == 1){
            bot.intake.stopCollect();
        } else if (drop.isReleased() && servoDirection == -1){
            bot.intake.stopCollect();
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
            bot.lifter.goToPos(Range.clip((int) (lifterInitial + deltaLifter), 0, 2200));
        } else {
            lifterInManual = 0;
            deltaLifter = 0;
        }
    }

    public static int armInManual = 0, armInitial = 0;
    public static void controlArmManually2(double movement){
        if(bot.arm.isArmBusy()){
            armInitial = 0;
            armInManual = 0;
            return;
        }

        if(Math.abs(movement) > ARM_MANUAL_THRESH){
            armInManual += 1;
            if(armInManual == 1){
                armInitial = bot.arm.getArmPosition();
            }
            deltaArm += movement * ARM_MANUAL_WEIGHT;
            bot.arm.setArmTarget(Range.clip((int)(armInitial + deltaArm), -10, 1900));
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
            bot.arm.setArmTarget(bot.arm.getArmPosition() + (int)(deltaArm));
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
            bot.arm.setArmTarget(Arm.ArmPositions.HANG);
            bot.lifter.goToPos(Lifter.LifterStates.MID);
            bot.intake.toAngle(Intake.AngleAdjustStates.NEUTRAL);
            bot.intake.stopCollect();
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
