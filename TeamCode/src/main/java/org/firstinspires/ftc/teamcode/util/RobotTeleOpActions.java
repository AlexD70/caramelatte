package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Button;
import org.firstinspires.ftc.teamcode.lib.Controller;

public class RobotTeleOpActions {
    public static Robot bot;
    public static ElapsedTime timer = new ElapsedTime();

    public static void initActions(){
        bot = Robot.getRobotInstance(null);
    }

    public static void start(){
        timer.reset();
        bot.setState(Robot.RobotTeleOpStates.RUNNING);
    }

    public static void drive(@NonNull Controller ctrl, double weight){
        bot.drive.setDrivePowerWeighted(new Pose2d(-ctrl.rightStickX, ctrl.rightStickX, -ctrl.leftStickY), weight);
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
            bot.intake.toAngle(Intake.AngleAdjustStates.PLACE_POS);
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

    public static double LIFTER_MANUAL_WEIGHT = 18, LIFTER_MANUAL_THRESH = 0.3;
    public static void controlLifterManually(double movement){
        if(bot.lifter.isBusy()){
            return;
        }

        if(Math.abs(movement) > LIFTER_MANUAL_THRESH) {
            bot.lifter.goToPos((int) (movement * LIFTER_MANUAL_WEIGHT) + bot.lifter.getPos());
        }
    }

    public static void hangingModeInit(boolean button){
        if(timer.seconds() < 120){
            return;
        }

        if(button){
            bot.setState(Robot.RobotTeleOpStates.HANGING_MODE);
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
        }
    }

    public static double LIFTER_HANGING_WEIGHT = 30, LIFTER_HANGING_THRESH = 0.5;
    public static void lifterHangingOverride(double movement){
        if(bot.isHanged && Math.abs(movement) > LIFTER_HANGING_THRESH){
            bot.lifter.goToPos(bot.lifter.getPos() + (int)(movement * LIFTER_HANGING_WEIGHT));
        }
    }

    public static void launchPlane(boolean button){
        if(timer.seconds() < 120){
            return;
        }

        if(button){
            bot.launcher.launchPlane();
        }
    }
}
