package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class DeadWheelOdometry extends ThreeTrackingWheelLocalizer {
    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public DeadWheelOdometry(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, RRConstants.LATERAL_DISTANCE_ODOMETRY / 2, 0), // left
                new Pose2d(0, -RRConstants.LATERAL_DISTANCE_ODOMETRY / 2, 0), // right
                new Pose2d(RRConstants.FORWARD_OFFSET_ODOMETRY, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RRConstants.odometry[0]));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RRConstants.odometry[1]));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RRConstants.odometry[2]));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
//        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);
//        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return RRConstants.WHEEL_RADIUS_ODOMETRY * 2 *
                Math.PI * RRConstants.GEAR_RATIO_ODOMETRY * ticks /
                RRConstants.TICKS_PER_REV_ODOMETRY;
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * RRConstants.X_MULTIPLIER_ODOMETRY,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * RRConstants.X_MULTIPLIER_ODOMETRY,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * RRConstants.Y_MULTIPLIER_ODOMETRY
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * RRConstants.X_MULTIPLIER_ODOMETRY,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * RRConstants.X_MULTIPLIER_ODOMETRY,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * RRConstants.Y_MULTIPLIER_ODOMETRY
        );
    }

    public void update(Pose2d correctedPose, Pose2d correctedVelocity){
        setPoseEstimate(correctedPose);
        setPoseVelocity(correctedVelocity);
    }
}