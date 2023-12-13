package org.firstinspires.ftc.teamcode.util;

import  com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    protected DcMotorEx m_armMotor;

    private int armPosition = 0, armTarget = 0, lastArmTarget = 0;
    private boolean armInManual = false; // manual actually means dont use encoders
    private double manualArmPower = 0;

    // ======================== ARM - MOTOR ========================

    public Arm(HardwareMap hwmap, String motorName){
        m_armMotor = hwmap.get(DcMotorEx.class, motorName);
    }

    public enum ArmPositions {
        INIT(0), COLLECT(0), PLACE(-800), PRELOAD_PLACE(-1500), MANUAL(-1), NO_ENCODER(-2);

        public int pos;

        ArmPositions(int pos) {
            this.pos = pos;
        }
    }
    private ArmPositions currentState;

    public ArmPositions getArmState(){
        return currentState;
    }

    public int getArmPosition(){
        return armPosition;
    }

    public boolean isArmBusy(){
        return m_armMotor.isBusy();
    }

    public void update(){
        armPosition = m_armMotor.getCurrentPosition();

        if(!armInManual) {
            if(armTarget != lastArmTarget){
                m_armMotor.setTargetPosition(armTarget);
                m_armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_armMotor.setPower(0.5);
            }
        } else {
            m_armMotor.setPower(manualArmPower);
        }
    }

    private void setArmTarget(int target){
        armTarget = target;
    }

    public void setArmTarget(ArmPositions target){
        setArmTarget(target.pos);
        currentState = target;
    }

    public void forceArmToPosition(int position){
        setArmTarget(position);
        currentState = ArmPositions.MANUAL;
    }

    public void noEncoderMode_setArmPower(double pow){
        manualArmPower = pow;
    }

    // once entered, there is no way to go back
    public void enterNoEncoderMode(){
        armInManual = true;
        currentState = ArmPositions.NO_ENCODER;
    }
}
