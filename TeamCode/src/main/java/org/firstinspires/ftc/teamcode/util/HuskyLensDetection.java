package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLensDetection {
    protected HuskyLens lens;

    private int currentId = -1;
    private HuskyLens.Algorithm currentAlgo = HuskyLens.Algorithm.NONE;
    public static final int PIXEL_ID = 1, CUSTOM_ELEMENT_ID = 2;

    private static final int XCOORD_LEFT = 90, XCOORD_RIGHT = 210;

    public HuskyLensDetection(HardwareMap hwmap, String name){
        lens = hwmap.get(HuskyLens.class, name);
        lens.initialize();
    }

    enum RandomisationCase {
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

    public RandomisationCase getRandomisationCase(int id){
        lens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        HuskyLens.Block[] blocks = lens.blocks(id);

        if(blocks.length == 0){
            return RandomisationCase.UNKNOWN;
        }

        if (blocks[0].x + blocks[0].width < XCOORD_LEFT) {
            return RandomisationCase.LEFT;
        } else if (blocks[0].x > XCOORD_RIGHT) {
            return RandomisationCase.RIGHT;
        } else {
            return RandomisationCase.CENTER;
        }
    }

    public void update(){
        lens.knock();
    }
}
