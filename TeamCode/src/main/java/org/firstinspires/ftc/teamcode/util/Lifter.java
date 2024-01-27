package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import androidx.core.util.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PID;
import org.firstinspires.ftc.teamcode.lib.PIDF;

public class Lifter {
    protected DcMotorEx m_left, m_right;

    // lower kP until lifter is no longer spasming around targetrget position
    private final double kP = 0.004d, kD = 0d, kI = 0.00001d;
    private final Supplier<Double> kF = () -> 0.02d;
    private final PIDF pidf = new PIDF(kP, kD, kI, kF);

    public Lifter(@NonNull HardwareMap hwmap){
        m_left = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_LEFT);
        m_right = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_RIGHT);

        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setDirection(DcMotorSimple.Direction.REVERSE);
        m_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void reset(){
        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setDirection(DcMotorSimple.Direction.REVERSE);
        m_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public enum LifterStates {
        INIT(0), DOWN(0), MID(1000), HIGH(1500), ULTRA_HIGH(2100), HANG(600), MANUAL(-1), NO_ENCODER(-2);

        public int pos;
        LifterStates(int pos){this.pos = pos;}
    }
    LifterStates lifterState = LifterStates.INIT;

    public int lastTarget = 0, target = 0, currentPosition = 0;
    public double power = 0;

    public int getPos(){
        return currentPosition;
    }

    public void goToPos(int x){
        lifterState = LifterStates.MANUAL;
        target = x;
    }
    public void goToPos(@NonNull LifterStates state){
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

    public void goDownExtraVoltage(){
        pidf.kP = 0.0028;
        pidf.kI = 0.00009;
        pidf.resetIntegral();
        goToPos(LifterStates.DOWN);
    }

    public double velocity = 0, acceleration = 0, lastVel = 0;
    public int lastPosition = 0, dtheta = 0;
    private ElapsedTime dtTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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

        if(currentPosition > 2400){
            m_left.setPower(0);
            m_right.setPower(0);
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

    public boolean isBusy(){
        return (Math.abs(currentPosition - target) > 20) && (lifterState != LifterStates.MANUAL);
    }

    public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Lifter position", currentPosition);
        telemetry.addData("Lifter target", target);
        telemetry.addData("Lifter power", power);
        telemetry.addData("Lifter state", 0);
        telemetry.addData("velocity", velocity);
        telemetry.addData("accel", acceleration);
    }
}