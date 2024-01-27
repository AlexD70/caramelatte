package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Lifter;
import org.firstinspires.ftc.teamcode.util.VoltageScaledArm;

@TeleOp(name = "test voltage scaled arm")
public class TestVoltageScaledArm extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageScaledArm arm = new VoltageScaledArm(hardwareMap);
        Lifter lift = new Lifter(hardwareMap);
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double accelMax = 0;
        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.dpad_up){
                lift.goToPos(2000);
            } else if (gamepad1.dpad_down){
                lift.goToPos(0);
            }

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

            double angvel = lift.velocity / 537.7 * 2 * Math.PI;
            double deltaTheta = lift.currentPosition / 537.7 * 2 * Math.PI;
            double accel = angvel * angvel * (35.65 / 2 * 1e-3) / (deltaTheta * 4/3);
            if(accel > accelMax){
                accelMax = accel;
            }

            telemetry.addData("max accel", accelMax);
            arm.update(multipleTelemetry);
            lift.update();
            lift.printDebug(multipleTelemetry);
            multipleTelemetry.update();
        }
    }
}
