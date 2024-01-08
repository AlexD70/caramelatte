package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {
    protected Servo s_launcher;

    public PlaneLauncher(@NonNull HardwareMap hwmap){
        s_launcher = hwmap.get(Servo.class, HardwareConfig.LAUNCHER);
        s_launcher.setPosition(0.6);
    }

    public void launchPlane(){
        s_launcher.setPosition(0);
    }
}
