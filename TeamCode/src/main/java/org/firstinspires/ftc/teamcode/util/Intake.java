package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    protected CRServo crs_leftGecko, crs_rightGecko;
    protected Servo s_angleAdjust;

    private Thread crsSchedulerThread = new Thread();

    public Intake(HardwareMap hwmap, String crs_left, String crs_right, String servo){
        crs_leftGecko = hwmap.get(CRServo.class, crs_left);
        crs_rightGecko = hwmap.get(CRServo.class, crs_right);
        s_angleAdjust = hwmap.get(Servo.class, servo);
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

    public enum AngleAdjustStates {
        INIT(0d), COLLECT_POS(0d), PLACE_POS(.3), MANUAL(-1);

        public double val;
        AngleAdjustStates(double val){this.val = val;}
    }
    private AngleAdjustStates currentState = AngleAdjustStates.INIT;

    public void toggleAngleAdjustPos(){
        if(currentState != AngleAdjustStates.PLACE_POS){
            s_angleAdjust.setPosition(AngleAdjustStates.PLACE_POS.val);
            currentState = AngleAdjustStates.PLACE_POS;
        } else {
            s_angleAdjust.setPosition(AngleAdjustStates.COLLECT_POS.val);
            currentState = AngleAdjustStates.COLLECT_POS;
        }
    }

    public void forceAngleServoPos(double pos){
        s_angleAdjust.setPosition(pos);
        currentState = AngleAdjustStates.MANUAL;
    }
}
