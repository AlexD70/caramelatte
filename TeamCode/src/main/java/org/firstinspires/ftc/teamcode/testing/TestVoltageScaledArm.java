package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;

@TeleOp(name = "test voltage scaled arm")
public class TestVoltageScaledArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageScaledArm arm = new VoltageScaledArm(hardwareMap);
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad2.square){
                arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT);
            }

            if(gamepad2.triangle){
                arm.setArmTarget(VoltageScaledArm.ArmPositions.COLLECT_FORCE_POSITIVE);
            }

            if(gamepad2.cross){
                arm.setArmTarget(VoltageScaledArm.ArmPositions.HANG);
            }

            if(gamepad2.circle){
                arm.setArmTarget(VoltageScaledArm.ArmPositions.PLACE);
            }

            arm.update(multipleTelemetry);
            arm.printDebug(multipleTelemetry);
            multipleTelemetry.update();
        }
    }
}
