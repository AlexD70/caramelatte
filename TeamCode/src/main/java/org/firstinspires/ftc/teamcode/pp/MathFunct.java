package org.firstinspires.ftc.teamcode.pp;
public class MathFunct {


    //Se asigura ca unghiul e mereu intre -180 si 180 grade
    public static double AngleWrap(double angle){
        while(angle < Math.PI){
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

}
