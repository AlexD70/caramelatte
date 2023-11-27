package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

public class MecanumDriveEx extends MecanumDrive {
    public ArrayList<DcMotorEx> motors = new ArrayList<>();
    public VoltageSensor voltageSensor;
    public HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(
            RRConstants.TRANSLATIONAL_PID,
            RRConstants.TRANSLATIONAL_PID,
            RRConstants.HEADING_PID,
            RRConstants.MAX_FOLLOWER_ERROR,
            RRConstants.FOLLOWER_TIMEOUT
    );
    public TrajectoryBuilder builder = new TrajectoryBuilder(RRConstants.INITIAL_BOT_POSE, RRConstants.VEL_CONSTRAINT, RRConstants.ACCEL_CONSTRAINT);
    public Localizer odometry = null;
    protected boolean isTurning = false, hasJustTurned = false;
    public PIDFController turnController = new PIDFController(RRConstants.HEADING_PID);

    public MecanumDriveEx(HardwareMap hwmap){
        super(RRConstants.kV, RRConstants.kA, RRConstants.kStatic, RRConstants.TRACK_WIDTH);

        voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);

        for(String s : RRConstants.motors){
            DcMotorEx newMotor = hwmap.get(DcMotorEx.class, s);
            newMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            newMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            MotorConfigurationType config = newMotor.getMotorType().clone();
            config.setAchieveableMaxRPMFraction(1);
            newMotor.setMotorType(config);

            motors.add(newMotor);
        }

        setLocalizer(odometry);
        turnController.setInputBounds(0, Math.PI * 2);
    }

    @Override
    protected double getRawExternalHeading() {
        throw new RuntimeException("Stub!");
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        throw new RuntimeException("Stub!");
    }

    @Override
    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        motors.get(0).setPower(lf);
        motors.get(1).setPower(lb);
        motors.get(2).setPower(rf);
        motors.get(3).setPower(rb);
    }

    public void setDrivePowerWeighted(Pose2d xyt, double weight){
        setDrivePower(xyt.times(weight));
    }

    public boolean isBusy(){
        if (isTurning) {
            return true;
        }
        return follower.isFollowing();
    }

    public void waitForIdle(){
        while (!Thread.currentThread().isInterrupted() && isBusy()){
            update();
        }
    }

    public void update(){
        updatePoseEstimate();
        DriveSignal signal = follower.update(odometry.getPoseEstimate(), odometry.getPoseVelocity());
        setDriveSignal(signal);
    }

    public Pose2d getLastPoseError(){
        if(isTurning || hasJustTurned){
            return new Pose2d(0, 0, turnController.getLastError());
        }
        return follower.getLastError();
    }

    public void renewBuilder(Pose2d initPose){
        builder = new TrajectoryBuilder(initPose, RRConstants.VEL_CONSTRAINT, RRConstants.ACCEL_CONSTRAINT);
        return;
    }

    public void followTrajectory(Trajectory trajectory){
        hasJustTurned = false;
        follower.followTrajectory(trajectory);
        waitForIdle();
        renewBuilder(trajectory.end());
    }

    public void followTrajectoryAsync(Trajectory trajectory){
        hasJustTurned = false;
        follower.followTrajectory(trajectory);
        renewBuilder(trajectory.end());
    }

    // timeout - ms
    public void followTrajectoryWithTimeout(Trajectory trajectory, long timeout){
        hasJustTurned = false;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        follower.followTrajectory(trajectory);
        while(!Thread.currentThread().isInterrupted() && isBusy() && timer.milliseconds() <= timeout){
            update();
        }

        renewBuilder(trajectory.end());
    }

    public Pose2d turn(double angle, Pose2d startPose){
        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPoseEstimate().getHeading(), 0.0, 0.0, 0.0),
                new MotionState(getPoseEstimate().getHeading() + angle, 0.0, 0.0, 0.0),
                RRConstants.MAX_ANG_VEL,
                RRConstants.MAX_ANG_ACCEL
        );

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        isTurning = true;

        while(isTurning) {
            updatePoseEstimate();
            double deltaTime = timer.seconds();

            MotionState targetState = turnProfile.get(deltaTime);
            turnController.setTargetPosition(targetState.getX());

            double correction = turnController.update(getPoseEstimate().getHeading());

            double targetOmega = targetState.getV();
            double targetAlpha = targetState.getA();

            DriveSignal driveSignal = new DriveSignal(
                    new Pose2d(0, 0, targetOmega + correction),
                    new Pose2d(0, 0, targetAlpha)
            );

            if (deltaTime >= turnProfile.duration()) {
                isTurning = false;
                driveSignal = new DriveSignal();
            }

            setDriveSignal(driveSignal);
        }

        hasJustTurned = true;

        Pose2d lastPose = new Pose2d(
                startPose.getX(), startPose.getY(),
                Angle.norm(startPose.getHeading() + angle)
        );

        renewBuilder(lastPose.plus(getLastPoseError()));
        return lastPose;
    }
}
