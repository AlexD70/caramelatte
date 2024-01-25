package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotTeleOpActions;

@TeleOp(name = "test imu")
public class TestIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        Robot bot = new Robot(hardwareMap);
        bot.initTeleOp();
        IMU.Parameters params = new IMU.Parameters(orientation);

        imu.initialize(params);

        waitForStart();
        Controller ctrl1 = new Controller(gamepad1);

        while(opModeIsActive() && !isStopRequested()){
            RobotTeleOpActions.drive(ctrl1, 1);
            telemetry.addData("theta", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
            telemetry.addData("accel", imu.getRobotAngularVelocity(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}
