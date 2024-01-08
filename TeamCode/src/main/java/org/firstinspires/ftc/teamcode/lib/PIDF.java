package org.firstinspires.ftc.teamcode.lib;

import androidx.core.util.Supplier;

@SuppressWarnings("unused")
public class PIDF extends PID {
    Supplier<Double> kFun = () -> 0d;

    public PIDF(double kP, double kI, double kD, Supplier<Double> kFun) {
        super(kP, kI, kD);
        this.kFun = kFun;
    }

    @Override
    public double update(int currentTicks){
        int e = targetPos - currentTicks;
        int delta_e = lastError - e;
        sumError += e;
        lastError = e;
        return (e * kP + delta_e * kD + sumError * kI + kFun.get());
    }
}
