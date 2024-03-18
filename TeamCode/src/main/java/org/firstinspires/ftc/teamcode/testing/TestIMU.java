package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive3;

@TeleOp
public class TestIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive3 drive = new SampleMecanumDrive3(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =  new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        IMU.Parameters params = new IMU.Parameters(orientation);

        imu.initialize(params);

        waitForStart();
        Controller ctrl1 = new Controller(gamepad1);

        ElapsedTime timer = new ElapsedTime();
        double maxAccel = 0, lastVel = 0;
        while(opModeIsActive() && !isStopRequested()){
            double vel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            maxAccel = Math.max(maxAccel, Math.abs(lastVel - vel)/timer.seconds());
            timer.reset();
            lastVel = vel;

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            drive.update();

            telemetry.addData("theta", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
            telemetry.addData("accel", imu.getRobotAngularVelocity(AngleUnit.RADIANS));
            telemetry.addData("max", maxAccel);
            telemetry.update();
        }
    }
}
