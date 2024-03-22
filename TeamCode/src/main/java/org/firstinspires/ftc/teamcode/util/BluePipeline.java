package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePipeline extends OpenCvPipeline {
    // 1 - stanga 0 - centru -2 - dreapta CLOSE
    boolean kill = true;
    int stack = 0;
    boolean close = false;

    public void startDetection(boolean blueClose){
        kill = false;
        close = blueClose;
    }

    public volatile int whichCase = -2;

    public int getCase(){
        return whichCase;
    }

    public void killThis(){
        kill = true;
    }

    @Override
    public Mat processFrame(Mat input) {

        if(kill){
            return input;
        }

        if(input.empty()){
            return input;
        }

        int crop = 80;
        if(!close){
            crop = 125;
        }
        Point topLeft = new Point(0, crop);
        Point bottomRight =  new Point(320, 240);
        Rect roi =  new Rect(topLeft, bottomRight);
        Mat croppedInput = new Mat(input, roi);


        Mat whiteObjs = new Mat();

        Imgproc.cvtColor(croppedInput, input, Imgproc.COLOR_RGB2HSV);

        Core.inRange(input, new Scalar(100, 150, 0), new Scalar(140, 255, 255), whiteObjs);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(whiteObjs, contours, whiteObjs, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect target = null;
        double area = 0;
        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);
            if(rect.width > 100){
                continue;
            }

            if(rect.area() < 90){
                continue;
            }

            if(rect.area() > area){
                area = rect.area();
                target = rect;
            }
        }

        if(target == null){
            whichCase = -2;
            return input;
        }

        if(close) {
            if (target.x < 110) {
                whichCase = 1;
            } else if (target.x > 250) {
                whichCase = -1;
            } else {
                whichCase = 0;
            }
        } else {
            if(target.x > 130){
                whichCase = -1;
            } else if (target.x < 130){
                whichCase = 0;
            } else if (target.area() < 50) {
                whichCase = 1;
            }
        }

        Imgproc.rectangle(input, target, new Scalar(0, 255, 0));

        //Imgproc.drawContours(whiteObjs, contours, -1, new Scalar(1));
        return input;
    }
}
