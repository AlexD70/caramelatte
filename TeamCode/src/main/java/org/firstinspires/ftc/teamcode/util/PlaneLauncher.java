package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {
    protected Servo launcher;

    public PlaneLauncher(HardwareMap hwmap, String name){
        launcher = hwmap.get(Servo.class, name);
        launcher.setPosition(0.6);
    }

    public void launchPlane(){
        launcher.setPosition(0);
    }
}
