package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControllerPID {
    public final double kP, kD, kI, kCos;
    private int targetPosition, currentPosition, sum = 0, deriv, lastError = 0;
    private double minPow = -1d, maxPow = 1d;

    public ArmControllerPID(double kP, double kD, double kI, double kCos) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.kCos = kCos;
    }

    public void resetSum(){
        sum = 0;
    }

    public void setTarget(int target){
        targetPosition = target;
    }

    public int getTarget(){
        return targetPosition;
    }

    public void setPowerLimits(double min, double max){
        minPow = min;
        maxPow = max;
    }

    public double update(int currentPos, double currentRadians, Telemetry telemetry){
        int e = targetPosition - currentPos;
        int delta_e = lastError - e;
        telemetry.addData("rad", currentRadians);
        telemetry.addData("cos", Math.cos(currentRadians));
        telemetry.addData("cos*k", kCos * Math.cos(currentRadians));

        sum += e;
        lastError = e;

        return Range.clip((kP * e + kD * delta_e + kI * sum) + kCos * Math.cos(currentRadians), minPow, maxPow);
    }
}
