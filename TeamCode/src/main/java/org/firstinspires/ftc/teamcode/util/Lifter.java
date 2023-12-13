package org.firstinspires.ftc.teamcode.util;

import androidx.core.util.Supplier;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.lib.PIDF;

public class Lifter {
    protected DcMotorEx left, right;

    private final double kP = 0.008d, kD = 0d, kI = 0.00001d;
    private final Supplier<Double> kF = () -> 0.02d;
    private final PIDF pidf = new PIDF(kP, kD, kI, kF);

    public Lifter(HardwareMap hwmap, String nameLeft, String nameRight){
        left = hwmap.get(DcMotorEx.class, nameLeft);
        right = hwmap.get(DcMotorEx.class, nameRight);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int lastTarget = 0, target = 0, currentPosition = 0;
    public double power = 0;

    public void goToPos(int x){
        target = x;
    }

    public void update(){
        currentPosition = left.getCurrentPosition();

        if(lastTarget != target){
            pidf.setTargetPosition(target);
            pidf.resetIntegral();
            lastTarget = target;
        }

        if(Math.abs(currentPosition - target) > 10){
            double pow = pidf.update(currentPosition);
            power = pow;
            right.setPower(pow);
            left.setPower(pow);
        }
    }
}