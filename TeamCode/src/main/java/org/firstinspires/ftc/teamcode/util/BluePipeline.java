package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePipeline extends OpenCvPipeline {
    boolean kill = true;
    int stack = 0;
    boolean close = false;

    public void startDetection(boolean blueClose){
        kill = false;
        close = blueClose;
    }

    public int whichCase = -2;

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

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Core.inRange(input, new Scalar(80, 50, 50), new Scalar(150, 255, 255), whiteObjs);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(whiteObjs, contours, whiteObjs, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect target = null;
        double area = 0;
        for(MatOfPoint contour : contours){
            Rect rect = Imgproc.boundingRect(contour);
            if(rect.area() > area){
                area = rect.area();
                target = rect;
            }
        }

        if(target == null){
            return input;
        }

        if(target.x + target.width < 110){
            whichCase = 1;
        } else if (target.x > 110 && target.x + target.width < 250){
            whichCase = 0;
        } else {
            whichCase = -1;
        }

        //Imgproc.drawContours(whiteObjs, contours, -1, new Scalar(1));
        return input;
    }
}
