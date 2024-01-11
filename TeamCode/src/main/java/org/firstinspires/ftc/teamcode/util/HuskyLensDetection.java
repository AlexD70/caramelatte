package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HuskyLensDetection {
    public HuskyLens lens;

    private int currentId = -1;
    private HuskyLens.Algorithm currentAlgo = HuskyLens.Algorithm.NONE;
    private boolean algoSet = false;
    public RandomisationCase lastKnownCase = RandomisationCase.UNKNOWN;
    public static final int PIXEL_ID = -1, CUSTOM_ELEMENT_ID = 1;

    private static final int XCOORD_LEFT = 90, XCOORD_RIGHT = 210;

    public HuskyLensDetection(HardwareMap hwmap, String name){
        lens = hwmap.get(HuskyLens.class, name);
        try {
            Thread.sleep(3000);
        } catch (Exception e){
            e.printStackTrace();
        }
        lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public static enum RandomisationCase {
        LEFT(-1), CENTER(0), RIGHT(1), UNKNOWN(100);

        public final int val;
        RandomisationCase(int val){this.val = val;}
    }

    private final Thread autoKnockThread = new Thread(() -> {
        while (!Thread.currentThread().isInterrupted()) {
            lens.knock();

            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                break;
            }
        }
    });

    public void startAutoKnock(){
        autoKnockThread.start();
    }

    public void stopAutoKnock(){
        autoKnockThread.interrupt();
    }

    public RandomisationCase getRandomisationCase(int id) throws InterruptedException {
        if (id == 0) { // blue far
            lens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

            HuskyLens.Block[] blocks = lens.blocks(1);

            //telemetry.addData("blocks", blocks.length);

            if (blocks.length == 0) {
                return lastKnownCase;
            }

            if (blocks[0].x > 100 && blocks[0].x + blocks[0].width < 180) {
                lastKnownCase = RandomisationCase.CENTER;
                return RandomisationCase.CENTER;
            } else if (blocks[0].x >= 180) {
                lastKnownCase = RandomisationCase.RIGHT;
                return RandomisationCase.RIGHT;
            } else if (blocks.length == 1) {
                lastKnownCase = RandomisationCase.LEFT;
                return RandomisationCase.LEFT;
            }
        } else if (id == 1) { // blue close
            lens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

            HuskyLens.Block[] blocks = lens.blocks(1);

            if (blocks.length == 0) {
                return lastKnownCase;
            }

            if (blocks[0].x > 100 && blocks[0].x + blocks[0].width < 180) {
                lastKnownCase = RandomisationCase.CENTER;
                return RandomisationCase.CENTER;
            } else if (blocks[0].x + blocks[0].width < 100) {
                lastKnownCase = RandomisationCase.LEFT;
                return RandomisationCase.LEFT;
            } else if (blocks.length == 1) {
                lastKnownCase = RandomisationCase.RIGHT;
                return RandomisationCase.RIGHT;
            }
        } else if (id == 2) { // red far
            lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

            HuskyLens.Block[] blocks = lens.blocks(1);

            if (blocks.length == 0) {
                return lastKnownCase;
            }

            if (blocks[0].x > 100 && blocks[0].x + blocks[0].width < 180) {
                lastKnownCase = RandomisationCase.CENTER;
                return RandomisationCase.CENTER;
            } else if (blocks[0].x + blocks[0].width < 100) {
                lastKnownCase = RandomisationCase.LEFT;
                return RandomisationCase.LEFT;
            } else if (blocks.length == 1) {
                lastKnownCase = RandomisationCase.RIGHT;
                return RandomisationCase.RIGHT;
            }
        } else {
            lens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

            HuskyLens.Block[] blocks = lens.blocks(1);

            if (blocks.length == 0) {
                return lastKnownCase;
            }

            if (blocks[0].x > 100 && blocks[0].x + blocks[0].width < 180) {
                lastKnownCase = RandomisationCase.CENTER;
                return RandomisationCase.CENTER;
            } else if (blocks[0].x + blocks[0].width < 100) {
                lastKnownCase = RandomisationCase.RIGHT;
                return RandomisationCase.RIGHT;
            } else if (blocks.length == 1) {
                lastKnownCase = RandomisationCase.LEFT;
                return RandomisationCase.LEFT;
            }
        }

        Thread.sleep(30);
        return lastKnownCase;
    }

    private RandomisationCase lastCase = RandomisationCase.UNKNOWN;
    private ElapsedTime timerBetweenDefaults = new ElapsedTime();
    private boolean firstCall = true;
    private static final double AREA_THRESH_RED_MIN = 180, AREA_THRESH_RED_MAX = 600;
    public RandomisationCase getCaseRedFar(Telemetry telemetry) throws InterruptedException{
        if(firstCall){
            timerBetweenDefaults.reset();
            firstCall = false;
        }
        Thread.sleep(50);

        HuskyLens.Block[] blocks = lens.blocks(2);
        telemetry.addData("LENGTH", blocks.length);

        HuskyLens.Block target = null;
        double maxArea = 0;
        for(HuskyLens.Block b : blocks){
            double area = b.width * b.height / 2d;
            if(area < AREA_THRESH_RED_MIN){
                continue;
            } else if (area > AREA_THRESH_RED_MAX){
                continue;
            } else if (area > maxArea){
                maxArea = area;
                target = b;
            }
        }

        if(target != null){
            timerBetweenDefaults.reset();
            if(target.x + target.width < 160){
                telemetry.addLine("LEFT");
                lastCase = RandomisationCase.LEFT;
            } else if (target.x > 120 && target.width < 50){
                telemetry.addLine("CENTER");
                lastCase = RandomisationCase.CENTER;
            }
        } else {
            if(timerBetweenDefaults.seconds() > 2.5){
                telemetry.addLine("RIGHT");
                lastCase = RandomisationCase.RIGHT;
                timerBetweenDefaults.reset();
            }
        }

        return lastCase;
    }

    private static final double AREA_THRESH_RED_CLOSE_MIN = 180, AREA_THRESH_RED_CLOSE_MAX = 600;
    public RandomisationCase getCaseRedClose(Telemetry telemetry) throws InterruptedException{
        if(firstCall){
            timerBetweenDefaults.reset();
            firstCall = false;
        }
        Thread.sleep(50);

        HuskyLens.Block[] blocks = lens.blocks(2);
        telemetry.addData("LENGTH", blocks.length);

        HuskyLens.Block target = null;
        double maxArea = 0;
        for(HuskyLens.Block b : blocks){
            double area = b.width * b.height / 2d;
            if(area < AREA_THRESH_RED_MIN){
                continue;
            } else if (area > AREA_THRESH_RED_MAX){
                continue;
            } else if (area > maxArea){
                maxArea = area;
                target = b;
            }
        }

        if(target != null){
            timerBetweenDefaults.reset();
            if(target.x > 140){
                telemetry.addLine("RIGHT");
                lastCase = RandomisationCase.RIGHT;
            } else if (target.x + target.width < 200){
                telemetry.addLine("CENTER");
                lastCase = RandomisationCase.CENTER;
            }
        } else {
            if(timerBetweenDefaults.seconds() > 2.5){
                telemetry.addLine("LEFT");
                lastCase = RandomisationCase.LEFT;
                timerBetweenDefaults.reset();
            }
        }

        return lastCase;
    }

    private static final double AREA_THRESH_BLUE_MIN = 180, AREA_THRESH_BLUE_MAX = 600;
    public RandomisationCase getCaseBlueFar(Telemetry telemetry) throws InterruptedException{
        if(firstCall){
            timerBetweenDefaults.reset();
            firstCall = false;
        }
        Thread.sleep(50);

        HuskyLens.Block[] blocks = lens.blocks(1);
        telemetry.addData("LENGTH", blocks.length);

        HuskyLens.Block target = null;
        double maxArea = 0;
        for(HuskyLens.Block b : blocks){
            double area = b.width * b.height / 2d;
            if(area < AREA_THRESH_BLUE_MIN){
                continue;
            } else if (area > AREA_THRESH_BLUE_MAX){
                continue;
            } else if (area > maxArea){
                maxArea = area;
                target = b;
            }
        }

        if(target != null){
            timerBetweenDefaults.reset();
            if(target.x + target.width > 200){
                telemetry.addLine("LEFT");
                lastCase = RandomisationCase.LEFT;
            } else if (target.x > 120 && target.x + target.width < 200){
                telemetry.addLine("CENTER");
                lastCase = RandomisationCase.CENTER;
            }
        } else {
            if(timerBetweenDefaults.seconds() > 2.5){
                telemetry.addLine("RIGHT");
                lastCase = RandomisationCase.RIGHT;
                timerBetweenDefaults.reset();
            }
        }

        return lastCase;
    }

    private static final double AREA_THRESH_BLUE_CLOSE_MIN = 180, AREA_THRESH_BLUE_CLOSE_MAX = 600;
    public RandomisationCase getCaseBlueClose(Telemetry telemetry) throws InterruptedException{
        if(firstCall){
            timerBetweenDefaults.reset();
            firstCall = false;
        }
        Thread.sleep(50);

        HuskyLens.Block[] blocks = lens.blocks(1);
        telemetry.addData("LENGTH", blocks.length);

        HuskyLens.Block target = null;
        double maxArea = 0;
        for(HuskyLens.Block b : blocks){
            double area = b.width * b.height / 2d;
            if(area < AREA_THRESH_BLUE_CLOSE_MIN){
                continue;
            } else if (area > AREA_THRESH_BLUE_CLOSE_MAX){
                continue;
            } else if (area > maxArea){
                maxArea = area;
                target = b;
            }
        }

        if(target != null){
            timerBetweenDefaults.reset();
            if(target.x + target.width < 140){
                telemetry.addLine("RIGHT");
                lastCase = RandomisationCase.RIGHT;
            } else if (target.x > 120 && target.x + target.width < 220){
                telemetry.addLine("CENTER");
                lastCase = RandomisationCase.CENTER;
            }
        } else {
            if(timerBetweenDefaults.seconds() > 2.5){
                telemetry.addLine("LEFT");
                lastCase = RandomisationCase.LEFT;
                timerBetweenDefaults.reset();
            }
        }

        return lastCase;
    }


    @Nullable
    public ArrayList<Rect> getBackdropTags(){
        lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        HuskyLens.Block[] blocks = lens.blocks();

        if(blocks.length == 0){
            return null;
        }

        ArrayList<Rect> arr = new ArrayList<>(3);
        for(HuskyLens.Block b : blocks){
            arr.add(new Rect(b.x, b.y, b.width, b.height));
        }

        return arr;
    }

    public void update(){
        lens.knock();
    }
}
