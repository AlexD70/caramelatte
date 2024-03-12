package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm implements Mechanism {
    protected Servo left_armServo;
    protected Servo right_armServo;

    public Arm(@NonNull HardwareMap hwmap){
        left_armServo = hwmap.get(Servo.class, HardwareConfig.ARM);
        right_armServo = hwmap.get(Servo.class, HardwareConfig.ARM);
        left_armServo.setPosition(0);
        right_armServo.setDirection(Servo.Direction.REVERSE);
        right_armServo.setPosition(0);
    }

    public enum ArmPositions {
        INIT(0), COLLECT(0), PLACE(0.8), MANUAL(-1);

        public double pos;

        ArmPositions(double pos) {
            this.pos = pos;
        }
    }
    private ArmPositions currentState = ArmPositions.INIT;
    private double currentPos = 0;

    public void setPosition(@NonNull ArmPositions position){
        setPosition(position.pos);
    }

    public void setPosition(double target){
        left_armServo.setPosition(target);
        right_armServo.setPosition(target);
        currentPos = target;
    }

    public double getPositionDouble(){
        return currentPos;
    }

    @Override
    public void update(){}

    @Override
    public int getPosition() {
        return (int) currentPos * 10;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setTarget(int target) {
        throw new RuntimeException("Use setTarget(double) for the Arm!");
    }

    @Override
   public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Arm position", currentPos);
        telemetry.addData("Arm state", currentState);
   }
}
