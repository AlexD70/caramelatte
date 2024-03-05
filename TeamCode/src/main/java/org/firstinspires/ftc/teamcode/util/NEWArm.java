package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.VoltageScaledCosPID;

public class NEWArm {
    protected Servo left_armServo;
    protected Servo right_armServo;


    private double armPosition = 0, armTarget = 0, lastArmTarget = 0;
    private final double kP = 0.00469, kD = 0.0008, kI = 0, kCos = 0.016;
    private VoltageScaledCosPID pid = null;
    private boolean armInManual = false, isBusy = false; // manual actually means dont use encoders
    private double manualArmPower = 0;

    // ======================== ARM - SERVO ========================

    public NEWArm(@NonNull HardwareMap hwmap){
        left_armServo = hwmap.get(Servo.class, HardwareConfig.ARM);
        right_armServo = hwmap.get(Servo.class, HardwareConfig.ARM);
        left_armServo.setPosition(0);
        right_armServo.setDirection(Servo.Direction.REVERSE);
        right_armServo.setPosition(0);
        left_armServo.setDirection(Servo.Direction.FORWARD);
        right_armServo.setDirection(Servo.Direction.REVERSE);
        pid = new VoltageScaledCosPID(kP, kD, kI, kCos, hwmap.getAll(VoltageSensor.class).get(0));
    }

    public enum NewArmPositions {
        INIT(0), COLLECT(0), PLACE(1), PRELOAD_PLACE(1), PLACE_AUTO(0.8), HANG(0.5), MANUAL(-1), NO_ENCODER(-2);

        public double pos;

        NewArmPositions(double pos) {
            this.pos = pos;
        }
    }
    private NewArmPositions currentState = NewArmPositions.INIT;

    private int deltaTicks = 0;
    public void setStartPosition(int pos){
        deltaTicks = pos;
    }

    public final static double TICKS_TO_RAD = Math.PI * 2 / (28 * 103.8), INIT_RAD = -Math.PI/3;
    public double getApproximateAngle(){
        return TICKS_TO_RAD * getArmPosition() + INIT_RAD;
    }

    public NewArmPositions getArmState(){
        return currentState;
    }

    public double getArmPosition(){
        return armPosition;
    }

    public boolean isArmBusy(){
        return isBusy && currentState != NewArmPositions.MANUAL;
    }

    private double ticksOnLastUpdateCall = armPosition;
    private ElapsedTime timer = new ElapsedTime();
    public void update(Telemetry telemetry){
        armPosition = left_armServo.getPosition();

        if(!armInManual) {
            if(armTarget != lastArmTarget){
                left_armServo.setPosition(armTarget);
                right_armServo.setPosition(armTarget);
                isBusy = true;
                timer.reset();

                lastArmTarget = armTarget;
            }

            if(ticksOnLastUpdateCall == armPosition && timer.seconds() > 1){
                isBusy = false; // fix busy function to return false if arm not moving at all
                timer.reset();
            }

        }

        ticksOnLastUpdateCall = armPosition;
    }

    public void setArmTarget(double target){
        if(armInManual){
            return;
        }
        armTarget = target;
        currentState = NewArmPositions.MANUAL;
    }

    public void setArmTarget(NewArmPositions target){
        if(armInManual){
            return;
        }
        armTarget = target.pos;
        currentState = target;
    }

    @Deprecated
    public void forceArmToPosition(int position){
        setArmTarget(position);
        currentState = NewArmPositions.MANUAL;
    }


    // once entered, there is no way to go back
    public void enterNoEncoderMode(){
        armInManual = true;
        currentState = NewArmPositions.NO_ENCODER;
    }

   public void printDebug(@NonNull Telemetry telemetry){
        telemetry.addData("Arm position", armPosition);
        telemetry.addData("Arm target", armTarget);
        telemetry.addData("Arm state", currentState);
   }
}
