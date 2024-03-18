package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import androidx.core.util.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDF;

public class Lifter implements Mechanism {
    protected DcMotorEx m_left, m_right;

    private final double kP = 0.0025d, kD = 0d, kI = 0.00035d;
    private final Supplier<Double> kF = () -> 0.042d;
    private final PIDF pidf = new PIDF(kP, kD, kI, kF);
    public boolean isInAuto = false;

    public Lifter(@NonNull HardwareMap hwmap){
        m_left = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_LEFT);
        m_right = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_RIGHT);

        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setDirection(DcMotorSimple.Direction.FORWARD);
        m_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reset(){
        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setDirection(DcMotorSimple.Direction.FORWARD);
        m_left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setAuto(){
        isInAuto = true;
    }

    public enum LifterStates {
        INIT(0), DOWN(0), MID(1000), HIGH(1500), ULTRA_HIGH(2100), HANG(600), MANUAL(-1), NO_ENCODER(-2);

        public int pos;
        LifterStates(int pos){this.pos = pos;}
    }
    LifterStates lifterState = LifterStates.INIT;

    public int lastTarget = 0, target = 0, currentPosition = 0;
    public double power = 0;

    public int getPosition(){
        return currentPosition;
    }

    public void setTarget(int x){
        lifterState = LifterStates.MANUAL;
        target = Math.min(x, 3400);
    }
    public void setTarget(@NonNull LifterStates state){
        lifterState = state;
        target = state.pos;
    }

    boolean keepDown = false;
    double downPow = -0.1;
    public void keepDown(){
        keepDown = true;
    }

    public void stopKeepDown(){
        keepDown = false;
    }

    @Deprecated public void goDownExtraVoltage(){
        pidf.kP = 0.0028;
        pidf.kI = 0.00009;
        pidf.resetIntegral();
        setTarget(LifterStates.MID);
    }


    public double velocity = 0, acceleration = 0, lastVel = 0;
    public int lastPosition = 0, dtheta = 0;
    private ElapsedTime dtTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void update(){
        double dt = dtTimer.seconds();
        lastPosition = currentPosition;
        currentPosition = m_left.getCurrentPosition();

        lastVel = velocity;
        dtheta = currentPosition - lastPosition;
        velocity = (currentPosition - lastPosition) / dt;
        acceleration = (velocity - lastVel) / dt;
        dtTimer.reset();

        if(keepDown){
            m_left.setPower(downPow);
            m_right.setPower(downPow);
            return;
        }

        if(lastTarget != target){
            pidf.setTargetPosition(target);
            pidf.resetIntegral();
            lastTarget = target;
        }

        if((Math.abs(currentPosition - target) > 20) || lifterState == LifterStates.MANUAL){
            double pow = pidf.update(currentPosition);
            power = pow;
            m_right.setPower(pow);
            m_left.setPower(pow);
        }
    }

    @Override
    public boolean isBusy(){
        return (Math.abs(currentPosition - target) > 50) && ((lifterState != LifterStates.MANUAL) || isInAuto);
    }

    @Override
    public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Lifter position", currentPosition);
        telemetry.addData("Lifter target", target);
        telemetry.addData("Lifter power", power);
        telemetry.addData("Lifter state", 0);
        telemetry.addData("velocity", velocity);
        telemetry.addData("accel", acceleration);
    }
}