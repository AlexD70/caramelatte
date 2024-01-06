package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
        bot.drive.setDrivePowerWeighted(new Pose2d(-ctrl.rightStickX, ctrl.leftStickX, -ctrl.leftStickY), weight);
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

    public static double LIFTER_MANUAL_WEIGHT = 18, LIFTER_MANUAL_THRESH = 0.2;
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

    public static double ARM_MANUAL_WEIGHT = 5, ARM_MANUAL_THRESH = 0.4;
    public static double deltaArm = 0;
    public static void controlArmManually(double movement){
        if(bot.arm.isArmBusy()){
            deltaArm = 0;
            return;
        }

        if(Math.abs(movement) > ARM_MANUAL_THRESH){
            deltaArm += movement * ARM_MANUAL_WEIGHT;
            bot.arm.setArmTarget(bot.arm.getArmPosition() + (int)(deltaArm));
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
}
