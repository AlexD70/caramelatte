package org.firstinspires.ftc.teamcode.lib;

@SuppressWarnings("unused")
public class Feedforward {
    public double kV = 0, kA = 0, kStatic = 0;

    public Feedforward(double kV, double kA, double kStatic){
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    public double update(double targetV, double targetA){
        return targetV * kV + targetA * kA + kStatic;
    }
}
