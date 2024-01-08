package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline {
    // april tag detector native object
    private final long detectorPtr;
    public final static double TAGSIZE = 1, FX = 0.5, FY = 0.5, CX = 1, CY = 1;

    public ArrayList<AprilTagDetection> detectedTags = null;

    // array of rect coords to crop image
    public int[] cropRect = null;

    // flag to signal end of april tag detection
    private boolean killAprilTags = false;
    private boolean startAprilTags = false;

    public AprilTagPipeline(float initDecimation, int[] cropRectCoords){
        super();

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, initDecimation, 2);
        this.cropRect = new int[4];
        System.arraycopy(cropRectCoords, 0, this.cropRect, 0, 4);

        if(this.detectorPtr == 0){
            killAprilTags = true;
        }
    }

    public AprilTagPipeline(float initDecimation) {
        super();

        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, initDecimation, 2);

        // something failed at april tag detector initialization in the native fn
        if (this.detectorPtr == 0) {
            killAprilTags = true;
        }
    }

    Mat colorStream = new Mat();

    private void detectAllTags(Mat input){
        if(input.empty()){
            this.detectedTags = null;
            return;
        }

        input.copyTo(colorStream);
        if(this.cropRect != null){
            Rect roi = new Rect(cropRect[0], cropRect[1], cropRect[2], cropRect[3]);
            input = input.submat(roi);
        }
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        detectedTags = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                detectorPtr, input, TAGSIZE, FX, FY, CX, CY
        );

        if(detectedTags.isEmpty()){
            detectedTags = null;
        }

        colorStream.copyTo(input);
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!killAprilTags && startAprilTags) {
            detectAllTags(input);
        }
        return input;
    }

    public ArrayList<AprilTagDetection> getDetectedTags(){
        if(detectedTags == null){
            return new ArrayList<>();
        } else {
            return detectedTags;
        }
    }

    public void startAprilTagDetection() {
        startAprilTags = true;
        killAprilTags = false;
    }


    public void stopAprilTagDetection() {
        killAprilTags = true;
        startAprilTags = false;
    }
}
