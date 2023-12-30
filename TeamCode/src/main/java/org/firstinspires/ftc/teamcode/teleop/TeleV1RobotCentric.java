package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.MiscActions;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotTeleOpActions;

@TeleOp(name = "V1 - untested", group = "teleop")
public class TeleV1RobotCentric extends LinearOpMode {
    Robot bot;
    Controller ctrl1, ctrl2;

    public void controller1Actions(){
        // DRIVE
        if(ctrl1.isLeftTriggerDown()) {
            RobotTeleOpActions.drive(ctrl1, 0.3); // slow

        } else if (ctrl1.isRightTriggerDown()){
            RobotTeleOpActions.drive(ctrl1, 0.7); // fast

        } else {
            RobotTeleOpActions.drive(ctrl1, 0.5); // normal
        }

        // PLANE
        RobotTeleOpActions.launchPlane(ctrl1.bumperRight.isPressed());
    }

    public void controller2Actions(){
        // COLLECT/PLACE STATES
        RobotTeleOpActions.toCollectState(ctrl2.circle.isPressed());
        RobotTeleOpActions.toPlaceState(ctrl2.cross.isPressed());

        // LIFTER CONTROL
        if(ctrl2.dpadUp.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.HIGH);
        } else if (ctrl2.dpadDown.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.DOWN);
        }
        RobotTeleOpActions.controlLifterManually(-ctrl2.leftStickX);

        // COLLECTING/EJECTING/DROPPING PIXELS
        // hope this works
        RobotTeleOpActions.controlIntakeManually(ctrl2.bumperLeft, ctrl2.bumperRight);
    }

    public void hangingModeActions(){
        RobotTeleOpActions.hangingModeInit(ctrl1.dpadUp.isPressed());
        RobotTeleOpActions.tryHang(ctrl1.dpadDown.isPressed());
        RobotTeleOpActions.cancelHanging(ctrl1.cross.isPressed());
        RobotTeleOpActions.lifterHangingOverride(-ctrl2.rightStickX);

        // PLANE ALSO WHILE HANGING
        RobotTeleOpActions.launchPlane(ctrl1.bumperRight.isPressed());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Robot.getRobotInstance(hardwareMap);
        bot.initTeleOp();
        bot.setTelemetry(telemetry);
        ctrl1 = new Controller(gamepad1);
        ctrl2 = new Controller(gamepad2);
        MiscActions.bulkSetThreshes(ctrl1, ctrl2, 0.3);

        waitForStart();
        RobotTeleOpActions.start();

        while(opModeIsActive() && !isStopRequested()){
            ctrl1.update();
            ctrl2.update();
            bot.update();
            Robot.RobotTeleOpStates botState = bot.getTeleOpState();

            if (botState == Robot.RobotTeleOpStates.NORMAL) {
                controller1Actions();
                controller2Actions();
            } else if (botState == Robot.RobotTeleOpStates.HANGING_MODE) {
                if (!bot.isHanged) {
                    controller1Actions();
                }
                hangingModeActions();
            }
        }
    }
}
