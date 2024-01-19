package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.MiscActions;
import org.firstinspires.ftc.teamcode.util.Robot;
//import org.firstinspires.ftc.teamcode.util.RobotAutoActions;
import org.firstinspires.ftc.teamcode.util.RobotTeleOpActions;

@TeleOp(name = "TeleOp V2 - UNSTABLE", group = "teleop")
public class TeleV2RobotCentric extends LinearOpMode {
    Robot bot;
    Controller ctrl1, ctrl2;

    public void controller1Actions(){
        // DRIVE
        if(ctrl1.bumperLeft.isDown()) {
            RobotTeleOpActions.drive2(ctrl1, 0.6); // slow

        } else {
            RobotTeleOpActions.drive2(ctrl1, 1); // normal
        }

        // PLANE
        //RobotTeleOpActions.driveToPlaneLaunchZone(ctrl1.share.isPressed());
        RobotTeleOpActions.launchPlane(ctrl1.bumperRight.isPressed());
        RobotTeleOpActions.hangingModeInit(ctrl1.dpadUp.isPressed());
        if(ctrl1.dpadDown.isPressed()){
            RobotTeleOpActions.hangRobot();
        }
        if(ctrl1.square.isPressed()){
            RobotTeleOpActions.driveToPlaneLaunchZone();
        }
        RobotTeleOpActions.controlIntakeManually(ctrl1.leftTriggerButton, ctrl1.rightTriggerButton, true);
    }

    public void controller2Actions(){
        // COLLECT/PLACE STATES
        RobotTeleOpActions.toCollectState(ctrl2.circle.isPressed());
        RobotTeleOpActions.toPlaceState(ctrl2.cross.isPressed());

        // LIFTER CONTROL
        RobotTeleOpActions.controlLifter(ctrl2);
        RobotTeleOpActions.controlLifterManually2(-ctrl2.leftStickY);
        RobotTeleOpActions.controlArmManually2(ctrl2.rightStickY);

        if(ctrl2.square.isPressed()){
            bot.intake.toAngle(Intake.AngleAdjustStates.NEUTRAL);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap);
        bot.initTeleOpV2(hardwareMap);
        bot.setTelemetry(telemetry);
        ctrl1 = new Controller(gamepad1);
        ctrl2 = new Controller(gamepad2);
        MiscActions.bulkSetThreshes(ctrl1, ctrl2, 0.3);

        waitForStart();
        RobotTeleOpActions.start();

        while(opModeIsActive() && !isStopRequested()){
            MiscActions.bulkUpdate(ctrl1, ctrl2, bot, telemetry);

            bot.lifter.printDebug(telemetry);
            bot.arm.printDebug(telemetry);

            controller1Actions();
            controller2Actions();
        }
    }
}
