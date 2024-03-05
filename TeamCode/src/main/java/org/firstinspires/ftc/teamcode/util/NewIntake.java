package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewIntake {
    protected Servo s_angleAdjust_broom;
    protected DcMotorEx broom;
    private Thread crsSchedulerThread = new Thread();

    public NewIntake(@NonNull HardwareMap hwmap){
        broom = hwmap.get(DcMotorEx.class, HardwareConfig.BROOM);
        s_angleAdjust_broom = hwmap.get(Servo.class, HardwareConfig.ANGLE_ADJUST_BROOM);
    }

    // ====================== BROOM =====================

    public void startCollect(){
        broom.setPower(0.5);
    }

    public void startEject(){broom.setPower(-0.5);}

    public void stopCollect(){
        broom.setPower(0);
    }

    public static ElapsedTime timer = new ElapsedTime();

    public void dropPixel() throws InterruptedException {
        broom.setPower(-0.5);
        timer.reset();
        if(timer.seconds() > 2){
            broom.setPower(0);
        }
    }


    // ====================== ANGLE ADJUST =====================

    public enum AngleAdjustStates_BROOM {
        INIT(0d), COLLECT_POS(0.3), NEUTRAL(.8), MANUAL(-1);

        public double val;
        AngleAdjustStates_BROOM(double val){this.val = val;}
    }
    private AngleAdjustStates_BROOM broomCurrentState = AngleAdjustStates_BROOM.INIT;

    public void broomToAngle(AngleAdjustStates_BROOM state){
        s_angleAdjust_broom.setPosition(state.val);
        broomCurrentState = state;
    }

}
