package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;
import androidx.core.util.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDF;

public class Lifter {
    protected DcMotorEx m_left, m_right;

    private final double kP = 0.008d, kD = 0d, kI = 0.00001d;
    private final Supplier<Double> kF = () -> 0.02d;
    private final PIDF pidf = new PIDF(kP, kD, kI, kF);

    public Lifter(@NonNull HardwareMap hwmap){
        m_left = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_LEFT);
        m_right = hwmap.get(DcMotorEx.class, HardwareConfig.LIFTER_RIGHT);

        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum LifterStates {
        INIT(0), DOWN(10), MID(1000), HIGH(1500), MANUAL(-1), NO_ENCODER(-2);

        public int pos;
        LifterStates(int pos){this.pos = pos;}
    }
    LifterStates lifterState = LifterStates.INIT;

    public int lastTarget = 0, target = 0, currentPosition = 0;
    public double power = 0;

    public void goToPos(int x){
        lifterState = LifterStates.MANUAL;
        target = x;
    }
    public void goToPos(@NonNull LifterStates state){
        lifterState = state;
        target = state.pos;
    }

    public void update(){
        currentPosition = m_left.getCurrentPosition();

        if(lastTarget != target){
            pidf.setTargetPosition(target);
            pidf.resetIntegral();
            lastTarget = target;
        }

        if(Math.abs(currentPosition - target) > 10){
            double pow = pidf.update(currentPosition);
            power = pow;
            m_right.setPower(pow);
            m_left.setPower(pow);
        }
    }

    public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Lifter position", currentPosition);
        telemetry.addData("Lifter target", target);
        telemetry.addData("Lifter power", power);
        telemetry.addData("Lifter state", 0);
    }
}