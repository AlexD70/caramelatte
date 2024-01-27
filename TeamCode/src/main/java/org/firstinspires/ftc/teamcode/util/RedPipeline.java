package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedPipeline extends OpenCvPipeline {
    public boolean isFar = false;
    public boolean isStarted = false;
    public int detectionCase = -2;
    public RedPipeline(boolean isFar){
        this.isFar = isFar;
    }


    @Override
    public Mat processFrame(Mat input) {
        if(!isStarted){
            return input;
        }
        Mat copy = new Mat(320, 240, CvType.CV_32F);
        input.copyTo(copy);

        Imgproc.medianBlur(input, input, 3);
        Core.inRange(input, new Scalar(255, 55, 0), new Scalar(255, 255, 10), input);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, input, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_TC89_L1);

        MatOfPoint largestContour = null;
        double maxSize = 0;
        for(MatOfPoint contour : contours){
            double area = Imgproc.contourArea(contour);
            if (maxSize < area){
                maxSize = area;
                largestContour = contour;
            }
        }

        if(largestContour == null) {
            return copy;
        }

        Rect boundingRect = Imgproc.boundingRect(largestContour);
        if(isFar){
            if(boundingRect.x + boundingRect.width < 160){
                detectionCase = -1;
            } else if(boundingRect.x > 140 && (boundingRect.x + boundingRect.width < 240)){
                detectionCase = 0;
            } else {
                detectionCase = 1;
            }
        } else {
            if(boundingRect.x > 220){
                detectionCase = 1;
            } else if(boundingRect.x > 140 && (boundingRect.x + boundingRect.width < 240)){
                detectionCase = 0;
            } else {
                detectionCase = -1;
            }
        }
        return copy;
    }
}
