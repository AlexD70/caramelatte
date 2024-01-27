package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StackPipeline extends OpenCvPipeline {
    boolean kill = true;
    int stack = 0;
    boolean red = false;

    public void startDetection(int whichStack, boolean isRed){
        kill = false;
        stack = whichStack;
        red = isRed;
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
        Core.inRange(input, new Scalar(75, 0, 99), new Scalar(179, 62, 255), whiteObjs);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(whiteObjs, contours, whiteObjs, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Rect> targets = new ArrayList<>();
        for(MatOfPoint contour : contours){
            targets.add(Imgproc.boundingRect(contour));
        }

        //Imgproc.drawContours(whiteObjs, contours, -1, new Scalar(1));
        return whiteObjs;
    }
}
