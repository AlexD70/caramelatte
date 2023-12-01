package org.firstinspires.ftc.teamcode.util;

import androidx.core.util.Supplier;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.lib.PIDF;

public class Lifter {
    protected DcMotorEx left, right;

    private final double kP = 1d, kD = 0d, kI = 0d;
    private final Supplier<Double> kF = () -> 0d;
    private final PIDF pidf = new PIDF(kP, kD, kI, kF);

    public Lifter(HardwareMap hwmap, String nameLeft, String nameRight){
        left = hwmap.get(DcMotorEx.class, nameLeft);
        right = hwmap.get(DcMotorEx.class, nameRight);
    }
}
