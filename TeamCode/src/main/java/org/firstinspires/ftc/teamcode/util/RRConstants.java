package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class RRConstants {
    public static final double TICKS_PER_REV_ODOMETRY = 8192;
    public static final double WHEEL_RADIUS_ODOMETRY = 1; // in
    public static final double GEAR_RATIO_ODOMETRY = 1; // wheel speed / encoder speed

    public static final double LATERAL_DISTANCE_ODOMETRY = 12.8; //in
    public static final double FORWARD_OFFSET_ODOMETRY = 4.92; // in

    public static double X_MULTIPLIER_ODOMETRY = 1;
    public static double Y_MULTIPLIER_ODOMETRY = 1;

    public static String[] odometry = {"LF", "RB", "LB"}; // left, right, perpendicular

    public static String[] motors = {"LF", "LB", "RF", "RB"};

    public static final double TICKS_PER_REV = 8192;
    public static final double MAX_RPM = 312;

    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 18; // in

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 60;
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = 4.7601;
    public static double MAX_ANG_ACCEL = 3;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static Pose2d MAX_FOLLOWER_ERROR = new Pose2d(1, 1, Math.toRadians(0.5));
    public static double FOLLOWER_TIMEOUT = 0.5;

    public static TrajectoryVelocityConstraint VEL_CONSTRAINT = new MinVelocityConstraint(
            new ArrayList<TrajectoryVelocityConstraint>(
                    Arrays.asList(
                        new AngularVelocityConstraint(MAX_ANG_VEL),
                        new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
                    )
            )
    );
    public static TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = new MinAccelerationConstraint(
            new ArrayList<TrajectoryAccelerationConstraint>(
                    Collections.singletonList(
                            new ProfileAccelerationConstraint(MAX_ACCEL)
                    )
            )
    );
    public static Pose2d INITIAL_BOT_POSE = new Pose2d(0, 0, Math.toRadians(0));

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}
