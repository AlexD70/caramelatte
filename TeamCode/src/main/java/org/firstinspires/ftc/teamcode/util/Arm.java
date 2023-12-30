package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import  com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.ArmControllerPID;

public class Arm {
    protected DcMotorEx m_armMotor;

    private int armPosition = 0, armTarget = 0, lastArmTarget = 0;
    private final double kP = 0.0035, kD = 0, kI = 0, kCos = 0.1;
    private final ArmControllerPID pid = new ArmControllerPID(kP, kD, kI, kCos);
    private boolean armInManual = false, isBusy = false; // manual actually means dont use encoders
    private double manualArmPower = 0, power = 0;

    // ======================== ARM - MOTOR ========================

    public Arm(@NonNull HardwareMap hwmap){
        m_armMotor = hwmap.get(DcMotorEx.class, HardwareConfig.ARM);
        m_armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_armMotor.setDirection(DcMotorSimple.Direction.REVERSE); // use this to have positive state positions

        pid.setPowerLimits(-0.7, 0.7);
    }

    public enum ArmPositions {
        INIT(0), COLLECT(0), PLACE(1800), PRELOAD_PLACE(1800), HANG(1200), MANUAL(-1), NO_ENCODER(-2);

        public int pos;

        ArmPositions(int pos) {
            this.pos = pos;
        }
    }
    private ArmPositions currentState;

    public final static double TICKS_TO_RAD = Math.PI * 2 / (28 * 103.8), INIT_RAD = -Math.PI/3;
    public double getApproximateAngle(){
        return TICKS_TO_RAD * armPosition + INIT_RAD;
    }

    public ArmPositions getArmState(){
        return currentState;
    }

    public int getArmPosition(){
        return armPosition;
    }

    public boolean isArmBusy(){
        return isBusy;
    }

    private int ticksOnLastUpdateCall = armPosition;
    private ElapsedTime timer = new ElapsedTime();
    public void update(Telemetry telemetry){
        armPosition = m_armMotor.getCurrentPosition();

        if(!armInManual) {
            if(armTarget != lastArmTarget){
                pid.setTarget(armTarget);
                pid.resetSum();
                isBusy = true;
                timer.reset();

                lastArmTarget = armTarget;
            }

            if(ticksOnLastUpdateCall == armPosition && timer.seconds() > 1.5){
                isBusy = false; // fix busy function to return false if arm not moving at all
                timer.reset();
            }

            if(Math.abs(armTarget - armPosition) > 20 && isBusy){
                double pow = pid.update(armPosition, getApproximateAngle(), telemetry);
                power = pow;
                m_armMotor.setPower(pow);
            } else if (Math.abs(armTarget - armPosition) < 20){
                isBusy = false;
                power = 0;
                m_armMotor.setPower(0);
            }
        } else {
            m_armMotor.setPower(manualArmPower);
            power = manualArmPower;
        }

        ticksOnLastUpdateCall = armPosition;
    }

    private void setArmTarget(int target){
        if(armInManual){
            return;
        }
        armTarget = target;
    }

    public void setArmTarget(ArmPositions target){
        if(armInManual){
            return;
        }
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

   public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Arm position", armPosition);
        telemetry.addData("Arm target", armTarget);
        telemetry.addData("Arm power", power);
        telemetry.addData("Arm state", currentState);
   }
}
