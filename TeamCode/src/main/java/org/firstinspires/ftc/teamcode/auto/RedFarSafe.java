package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Arm;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class RedFarSafe extends LinearOpMode {
    Robot bot = new Robot();

    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.lift.setAuto();
        waitForStart();
        bot.drive.setPoseEstimate(new Pose2d(62,-34,Math.toRadians(0)));
        bot.drive.followTrajectorySequence(
        bot.drive.trajectorySequenceBuilder(new Pose2d(62,-34,Math.toRadians(0)))
                .lineTo(new Vector2d(11,-32))
                .splineToLinearHeading(new Pose2d(11,-25,Math.toRadians(90)),Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(11,50))
                .build()
);
    }
}
