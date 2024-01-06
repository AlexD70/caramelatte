package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HuskyLensDetection {
    protected HuskyLens lens;

    private int currentId = -1;
    private HuskyLens.Algorithm currentAlgo = HuskyLens.Algorithm.NONE;
    public RandomisationCase lastKnownCase = RandomisationCase.UNKNOWN;
    public static final int PIXEL_ID = -1, CUSTOM_ELEMENT_ID = 1;

    private static final int XCOORD_LEFT = 90, XCOORD_RIGHT = 210;

    public HuskyLensDetection(HardwareMap hwmap, String name){
        lens = hwmap.get(HuskyLens.class, name);
        lens.initialize();
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

    public RandomisationCase getRandomisationCase(int id) {
        if (id == 0) { // blue far
            lens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

            HuskyLens.Block[] blocks = lens.blocks(1);

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

        return  lastKnownCase;
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
