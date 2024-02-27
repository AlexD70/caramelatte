package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    protected CRServo crs_leftGecko, crs_rightGecko;
    protected Servo s_angleAdjust;

    private Thread crsSchedulerThread = new Thread();

    public Intake(@NonNull HardwareMap hwmap){
        crs_leftGecko = hwmap.get(CRServo.class, HardwareConfig.CRS_LEFT);
        crs_rightGecko = hwmap.get(CRServo.class, HardwareConfig.CRS_RIGHT);
        s_angleAdjust = hwmap.get(Servo.class, HardwareConfig.ANGLE_ADJUST);
        crs_leftGecko.setPower(0);
        crs_rightGecko.setPower(0);
    }

    // ===================== CRS ======================

    public void setCRSPowers(double power){
        crs_leftGecko.setPower(power);
        crs_rightGecko.setPower(-power);
    }


    public void startEject(){
        interruptCRSScheduler();
        setCRSPowers(-0.55);
    }


    //am modificat la collect, pt ca voiam sa vedem daca cu o discrepanta de puteri putem sa luam pixelii mai bine de jos,
    //in fact merge. Acum ia pixelii foarte constant direct in intake, fara sa trebuiasca sa dea afara si sa ia iar, also ia mai
    //bine din stackIntake

    public void startCollect(){
        interruptCRSScheduler();
//        setCRSPowers(.55);
        crs_leftGecko.setPower(0.1);
        crs_rightGecko.setPower(-1);
    }

    public void stopCollect(){
        interruptCRSScheduler();
        setCRSPowers(0);
    }

    public void startCollectFast(){
        interruptCRSScheduler();
        setCRSPowers(.9);
    }

    public void startEjectFast(){
        interruptCRSScheduler();
        setCRSPowers(-0.75);
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

    public void dropPixel() throws InterruptedException{
        interruptCRSScheduler();
        setCRSPowers(-0.5);
        Thread.sleep(500);
        setCRSPowers(0);
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
        INIT(0d), COLLECT_POS(0.3), PLACE_POS(.9), NEUTRAL(.8), MANUAL(-1);

        public double val;
        AngleAdjustStates(double val){this.val = val;}
    }
    private AngleAdjustStates currentState = AngleAdjustStates.INIT;

    public void toAngle(AngleAdjustStates state){
        s_angleAdjust.setPosition(state.val);
        currentState = state;
    }

    public void forceAngleServoPos(double pos){
        s_angleAdjust.setPosition(pos);
        currentState = AngleAdjustStates.MANUAL;
    }
}
