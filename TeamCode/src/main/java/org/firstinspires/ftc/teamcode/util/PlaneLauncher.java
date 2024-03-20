package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PlaneLauncher implements Mechanism{
    protected Servo s_launcher;

    public PlaneLauncher(@NonNull HardwareMap hwmap){
        s_launcher = hwmap.get(Servo.class, HardwareConfig.LAUNCHER);
        s_launcher.setPosition(0.4);
    }

    public void launchPlane(){
        s_launcher.setPosition(0.8);
    }

    @Override
    public int getPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setTarget(int target) {

    }

    @Override
    public void printDebug(Telemetry telemetry) {

    }

    @Override
    public void update() {

    }
}
