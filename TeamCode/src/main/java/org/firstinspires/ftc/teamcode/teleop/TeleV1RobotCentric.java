package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.MiscActions;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotTeleOpActions;

@TeleOp(name = "TeleOp V1 - functional", group = "teleop")
public class TeleV1RobotCentric extends LinearOpMode {
    Robot bot;
    Controller ctrl1, ctrl2;

    public void controller1Actions(){
        // DRIVE
        if(ctrl1.bumperLeft.isDown()) {
            RobotTeleOpActions.drive(ctrl1, 0.3); // slow

        } /*else if (ctrl1.isRightTriggerDown()){
            RobotTeleOpActions.drive(ctrl1, 1); // fast

        }*/ else {
            RobotTeleOpActions.drive(ctrl1, 1); // normal
        }

        // PLANE
        RobotTeleOpActions.launchPlane(ctrl1.bumperRight.isPressed());
        RobotTeleOpActions.hangingModeInit(ctrl1.dpadUp.isPressed());
        RobotTeleOpActions.controlIntakeManually(ctrl1.leftTriggerButton, ctrl1.rightTriggerButton);
        if(ctrl1.dpadDown.isPressed()){
            bot.lifter.goDownExtraVoltage();
            RobotTeleOpActions.killWheels();
        }
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
        } else if (ctrl2.dpadLeft.isPressed()){
            bot.lifter.goToPos(Lifter.LifterStates.ULTRA_HIGH);
        }
        RobotTeleOpActions.controlLifterManually2(-ctrl2.leftStickY);
        RobotTeleOpActions.controlArmManually2(ctrl2.rightStickY);
        RobotTeleOpActions.overrideLimits(ctrl2.share.isPressed());

        if(ctrl2.square.isPressed()){
            bot.intake.forceAngleServoPos(0.8);
        }
    }

    public void hangingModeActions(){
        RobotTeleOpActions.tryHang(ctrl1.dpadDown.isPressed());
        RobotTeleOpActions.cancelHanging(ctrl1.cross.isPressed());
        RobotTeleOpActions.lifterHangingOverride(-ctrl2.rightStickY);

        // PLANE ALSO WHILE HANGING
        RobotTeleOpActions.launchPlane(ctrl1.bumperRight.isPressed());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        bot.initTeleOpWithSensor(hardwareMap);
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
            telemetry.update();

            bot.lifter.printDebug(telemetry);
            bot.arm.printDebug(telemetry);

            controller1Actions();
            controller2Actions();
        }
    }
}
