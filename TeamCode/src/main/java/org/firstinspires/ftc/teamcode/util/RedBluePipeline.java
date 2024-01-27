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

public class RedBluePipeline extends OpenCvPipeline {
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

        Mat whiteObjs = new Mat();
        Mat redObjs = new Mat();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Imgproc.medianBlur(input, input, 5);

        int crop = 75;
        if(!close){
            crop = 125;
        }
        Point topLeft = new Point(0, crop);
        Point bottomRight =  new Point(320, 240);
        Rect roi =  new Rect(topLeft, bottomRight);
        Mat croppedInput = new Mat(input, roi);

        Core.inRange(croppedInput, new Scalar(0, 50, 45),  new Scalar(10, 255, 255), whiteObjs);
        Core.inRange(croppedInput, new Scalar(170, 50, 45), new Scalar(180, 255, 255), redObjs);

        Core.bitwise_or(whiteObjs, redObjs, whiteObjs);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(whiteObjs, contours, whiteObjs, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect target = null;
        double area = 0;
        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);
//            if(rect.width > 120){
//                continue;
//            }

            if(rect.height < 30 && close){
                continue;
            }

            if(rect.area() > area){
                area = rect.area();
                target = rect;
                rect.y += crop;
            }
        }

//        croppedInput.release();
        if(target == null){
            if(close){
                whichCase = 1;
            }
            return input;
        }

        if(close) {
            if (target.x < 180) {
                whichCase = 0;
            } else if (target.x > 180) {
                whichCase = -1;
            } else {
                whichCase = 1;
            }
        } else {
            if (target.x > 200) {
                whichCase = -1;
            } else if (target.x < 100) {
                whichCase = 1;
            } else if(target.area() < 150){
                whichCase = 1;
            } else {
                whichCase = 0;
            }
        }

        Imgproc.rectangle(input, target, new Scalar(0, 255, 0));

        //Imgproc.drawContours(whiteObjs, contours, -1, new Scalar(1));
        return input;
    }
}
