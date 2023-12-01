package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.lib.ArmControllerPID;

public class Arm {
    protected CRServo crs_leftGecko, crs_rightGecko;
    protected DcMotorEx m_armMotor;
    protected Servo s_angleAdjust;

    private final int ARM_TOLERANCE = 20;
    private final double ARM_MIN_POW = -0.5, ARM_MAX_POW = 0.5;
    private final double kP = 0.5, kD = 0, kI = 0, kCos = 0.03;
    private final ArmControllerPID pid = new ArmControllerPID(kP, kD, kI, kCos);
    private int armPosition = 0, armTarget = 0, lastArmTarget = 0;
    private boolean armIsBusy = false, armInManual = false; // manual actually means dont use encoders
    private double manualArmPower = 0;
    private Thread crsSchedulerThread = new Thread();

    // ======================== ARM - MOTOR ========================

    public Arm(HardwareMap hwmap, String leftCrsName, String rightCrsName, String servoName, String motorName){
        crs_leftGecko = hwmap.get(CRServo.class, leftCrsName);
        crs_rightGecko = hwmap.get(CRServo.class, rightCrsName);
        s_angleAdjust = hwmap.get(Servo.class, servoName);
        m_armMotor = hwmap.get(DcMotorEx.class, motorName);

        pid.setPowerLimits(ARM_MIN_POW, ARM_MAX_POW);
    }

    enum ArmPositions {
        INIT(0), COLLECT(0), PLACE(800), PRELOAD_PLACE(1500), MANUAL(-1), NO_ENCODER(-2);

        public final int pos;

        ArmPositions(int pos) {
            this.pos = pos;
        }
    }
    private ArmPositions currentState;

    public ArmPositions getArmState(){
        return  currentState;
    }

    public double positionToRadians(int position){
        return 0;
    }

    public int getArmPosition(){
        return armPosition;
    }

    public boolean isArmBusy(){
        return armIsBusy;
    }

    public void update(){
        armPosition = m_armMotor.getCurrentPosition();

        if(!armInManual) {
            if (lastArmTarget != armTarget) {
                pid.setTarget(armTarget);
                pid.resetSum();
                lastArmTarget = armTarget;
                armIsBusy = true;
            }

            if (Math.abs(armPosition - armTarget) > ARM_TOLERANCE) {
                m_armMotor.setPower(pid.update(armPosition, positionToRadians(armPosition)));
            } else {
                armIsBusy = false;
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

    // ===================== CRS ======================

    private void setCRSPowers(double power){
        crs_leftGecko.setPower(power);
        crs_rightGecko.setPower(-power);
    }

    public void startCollect(){
        interruptCRSScheduler();
        setCRSPowers(.5);
    }

    public void stopCollect(){
        interruptCRSScheduler();
        setCRSPowers(0);
    }

    public void dropBothPixels() {
        interruptCRSScheduler();
        crsSchedulerThread = new Thread(() -> {
            setCRSPowers(-0.5);

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            setCRSPowers(0);
        });

        crsSchedulerThread.start();
    }

    public void dropPixel() {
        interruptCRSScheduler();
        crsSchedulerThread = new Thread(() -> {
            setCRSPowers(-0.3);

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            setCRSPowers(0);
        });

        crsSchedulerThread.start();
    }

    public void interruptCRSScheduler() {
        crsSchedulerThread.interrupt();
        try {
            crsSchedulerThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // ====================== ANGLE ADJUST =====================

    enum AngleAdjustStates {
        INIT(0d), COLLECT_POS(0d), PLACE_POS(.3), MANUAL(-1);

        public double val;
        AngleAdjustStates(double val){this.val = val;}
    }
    private AngleAdjustStates angleState = AngleAdjustStates.INIT;

    public void toggleAngleAdjustPos(){
        if(angleState != AngleAdjustStates.PLACE_POS){
            s_angleAdjust.setPosition(AngleAdjustStates.PLACE_POS.val);
            angleState = AngleAdjustStates.PLACE_POS;
        } else {
            s_angleAdjust.setPosition(AngleAdjustStates.COLLECT_POS.val);
            angleState = AngleAdjustStates.COLLECT_POS;
        }
    }

    public void forceAngleServoPos(double pos){
        s_angleAdjust.setPosition(pos);
        currentState = ArmPositions.MANUAL;
    }

}
