package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MiscActions;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "test intake")
public class TestIntake extends LinearOpMode {
    Robot bot = new Robot();
    Controller ctrl1, ctrl2;

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        ctrl1 = new Controller(gamepad1);
        ctrl2 = new Controller(gamepad2);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            MiscActions.bulkUpdate(ctrl1, ctrl2, bot);
            MiscActions.bulkSetThreshes(ctrl1, ctrl2, 0.4);

            if(ctrl1.isRightTriggerPressed()){
                bot.intake.startCollect();
            } else if (ctrl1.isLeftTriggerPressed()){
                bot.intake.stopCollect();
            } else if (ctrl1.square.isPressed()){
                bot.intake.startEject();
            }

            if(ctrl1.dpadDown.isPressed()){
                bot.intake.setPosition(Intake.BroomStates.COLLECT_POS);
            } else if (ctrl1.dpadUp.isPressed()){
                bot.intake.setPosition(Intake.BroomStates.NEUTRAL);
            }
        }
    }
}
