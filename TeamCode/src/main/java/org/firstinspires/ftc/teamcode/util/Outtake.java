package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Mechanism{
    protected Servo s_rotateBox;
    protected Servo s_outtakeGear;
    protected Servo left_miniClaw;
    protected Servo right_miniClaw;

    public Outtake(@NonNull HardwareMap hwmap){
        left_miniClaw = hwmap.get(Servo.class, HardwareConfig.CLAW_LEFT);
        right_miniClaw = hwmap.get(Servo.class, HardwareConfig.CLAW_RIGHT);
        s_rotateBox = hwmap.get(Servo.class, HardwareConfig.BOX_ROTATION);
        s_outtakeGear = hwmap.get(Servo.class, HardwareConfig.GEAR_SERVO);
        catchPixels();
        s_rotateBox.setPosition(0.52);
        s_outtakeGear.setPosition(0.75);
    }


    // ====================== OUTTAKE CLAWS =====================

    public void dropBothPixels() {
        left_miniClaw.setPosition(1);
        right_miniClaw.setPosition(0.2);
    }
    public void catchPixels() {
        left_miniClaw.setPosition(0.3);
        right_miniClaw.setPosition(0.8);
    }

    public void dropLeftPixel() throws InterruptedException{
        left_miniClaw.setPosition(1);
    }

    public void dropRightPixel() throws InterruptedException{
        right_miniClaw.setPosition(0);
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
        throw new RuntimeException("The Outtake class doesnt support setTarget(int)!");
    }

    @Override
    public void printDebug(Telemetry telemetry) {

    }

    @Override
    public void update() {
        return;
    }

    // ====================== BOX ROTATION =====================

    public enum BoxRotationStates {
        INIT(.52), COLLECT_POS(.52), RIGHT(0.3), LEFT(0.74), MANUAL(-1);

        public double val;
        BoxRotationStates(double val){this.val = val;}
    }
    private BoxRotationStates currentBoxRotation = BoxRotationStates.INIT;

    public void rotateToAngle(BoxRotationStates state){
        s_rotateBox.setPosition(state.val);
        currentBoxRotation = state;
    }

    public void rotateToPos(double pos){
        s_rotateBox.setPosition(pos);
    }
    public void rotateToAngleManual(double pos){
        s_rotateBox.setPosition(pos);
        currentBoxRotation = BoxRotationStates.MANUAL;
    }

    // ====================== GEAR =====================

    public enum GearStates {
        INIT(0.65), COLLECT(0.75), TRANSITION(0), PLACE(0.15);

        public double val;
        GearStates(double val){this.val = val;}
    }
    private GearStates currentGearState = GearStates.INIT;
    private double currentGearPosition = 0.65;

    public void gearToPos(GearStates state){
        gearToPos(state.val);
        currentGearState = state;
    }

    public void gearToPos(double pos){
        s_outtakeGear.setPosition(pos);
        currentGearPosition = pos;
    }

}
