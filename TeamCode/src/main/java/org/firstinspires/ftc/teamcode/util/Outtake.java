package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    protected Servo s_outtakeAngleAdjust;
    protected Servo s_angleAdjust;
    protected Servo left_miniCLaw;
    protected Servo right_miniClaw;
    private Thread crsSchedulerThread = new Thread();

    public Outtake(@NonNull HardwareMap hwmap){
        left_miniCLaw = hwmap.get(Servo.class, HardwareConfig.CLAW_LEFT);
        right_miniClaw = hwmap.get(Servo.class, HardwareConfig.CLAW_RIGHT);
        s_outtakeAngleAdjust = hwmap.get(Servo.class, HardwareConfig.ANGLE_ADJUST_OUTTAKE);
        s_angleAdjust = hwmap.get(Servo.class, HardwareConfig.ANGLE_ADJUST);
        left_miniCLaw.setPosition(0);
        right_miniClaw.setPosition(0);
    }


    // ====================== OUTTAKE CLAWS =====================

    public void catchPixels() {
        left_miniCLaw.setPosition(1);
        right_miniClaw.setPosition(1);
    }
    public void dropBothPixels() {
        left_miniCLaw.setPosition(0);
        right_miniClaw.setPosition(0);
    }

    public void dropleftPixel() throws InterruptedException{
        left_miniCLaw.setPosition(0);
    }

    public void droprightPixel() throws InterruptedException{
        right_miniClaw.setPosition(0);
    }

    // ====================== OUTTAKE ANGLE ADJUST =====================

    public enum OUTTAKE_AngleAdjustStates {
        INIT(0d), COLLECT_POS(0.3), PLACE(.9), MANUAL(-1);

        public double val;
        OUTTAKE_AngleAdjustStates(double val){this.val = val;}
    }
    private OUTTAKE_AngleAdjustStates currentOuttakeState = OUTTAKE_AngleAdjustStates.INIT;

    public void outtakeToAngle(OUTTAKE_AngleAdjustStates state){
        s_outtakeAngleAdjust.setPosition(state.val);
        currentOuttakeState = state;
    }

    // ====================== ANGLE ADJUST =====================
    public enum AngleAdjustStates {
        INIT(0d), LEFT(0.3), RIGHT(.9), MANUAL(-1);

        public double val;
        AngleAdjustStates(double val){this.val = val;}
    }
    private AngleAdjustStates currentState = AngleAdjustStates.INIT;

    public void pixelToAngle(AngleAdjustStates state){
        s_angleAdjust.setPosition(state.val);
        currentState = state;
    }

}
