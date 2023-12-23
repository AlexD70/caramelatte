package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        StandardTrackingWheelLocalizer stwl = new StandardTrackingWheelLocalizer(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry);
        SampleMecanumDrive.telemetry = telemetry;
//        DcMotleftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lifterRight"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RB"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_x,
                            gamepad1.left_stick_x,
                            -gamepad1.left_stick_y
                    )
            );

            drive.update();

            if (gamepad1.circle)
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (gamepad1.triangle)
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Pose2d poseEstimate = drive.getPoseEstimate();



            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("position", stwl.getWheelPositions());
            telemetry.update();
        }
    }
}
