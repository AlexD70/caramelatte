package org.firstinspires.ftc.teamcode.pp;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pp.MathFunct.AngleWrap;

import org.firstinspires.ftc.teamcode.pp.MathFunct;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.drive.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.util.MecanumDriveEx;
import org.opencv.core.Mat;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

public class PpMovement {
    public static StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
    public static Pose2d myPose = myLocalizer.getPoseEstimate();
    public static MecanumDriveEx drive = new MecanumDriveEx(hardwareMap);
    public static void goToPosition(double x, double y, double movementSpeed){

        double distanceToPoint = Math.hypot(x - myPose.getX(), y -myPose.getY());

        double absoluteAngle = Math.atan2(y-myPose.getY(),x-myPose.getX());
        double relativeAngle = AngleWrap(absoluteAngle - myPose.getHeading());

        double relativeXtoPoint = Math.cos(relativeAngle) * distanceToPoint;
        double relativeYtoPoint = Math.sin(relativeAngle) * distanceToPoint;

        double movementXPower = relativeXtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double movementYPower = relativeYtoPoint / (Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));

        Pose2d drivePowers = new Pose2d(movementXPower,movementYPower,0);
        
        drive.setDrivePower(drivePowers);

    }
}
