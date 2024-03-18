package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Mechanism {
    protected Servo s_angleAdjust_broom;
    protected DcMotorEx broomMotor;
    private int broomRunning = 0;

    public Intake(@NonNull HardwareMap hwmap){
        broomMotor = hwmap.get(DcMotorEx.class, HardwareConfig.BROOM);
        s_angleAdjust_broom = hwmap.get(Servo.class, HardwareConfig.ANGLE_ADJUST_BROOM);
        stopCollect();
        s_angleAdjust_broom.setPosition(0);
    }

    public void startCollect(){
        broomMotor.setPower(-0.9);
        broomRunning = 1;
    }

    public void startEject(){
        broomMotor.setPower(0.9);
        broomRunning = -1;
    }

    public void stopCollect(){
        broomMotor.setPower(0);
        broomRunning = 0;
    }

    // METHOD IMPLEMENTATIONS

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
        telemetry.addData("broom state", broomCurrentState);
        telemetry.addData("broom pos", broomPos);
        telemetry.addData("broom running", broomRunning);
    }

    @Override
    public void update() {}

    // ====================== ANGLE ADJUST =====================

    public enum BroomStates {
        INIT(0.3), COLLECT_POS(.8), NEUTRAL(.3), MANUAL(-1);

        public double pos;
        BroomStates(double val){this.pos = val;}
    }
    private BroomStates broomCurrentState = BroomStates.INIT;
    private double broomPos = 0;

    public void setPosition(BroomStates state){
        setPosition(state.pos);
        broomCurrentState = state;
    }

    public void setPosition(double pos){
        s_angleAdjust_broom.setPosition(pos);
        broomPos = pos;
        broomCurrentState = BroomStates.MANUAL;
    }

}
