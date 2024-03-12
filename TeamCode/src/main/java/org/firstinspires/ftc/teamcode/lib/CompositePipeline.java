package org.firstinspires.ftc.teamcode.lib;

import androidx.core.util.Consumer;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.function.Function;

public class CompositePipeline extends OpenCvPipeline {
    protected Hashtable<String, Consumer<Mat>> processingSteps = new Hashtable<>();
    protected ArrayList<String> startedProcessors = new ArrayList<>();
    protected Function<Mat, Mat> mainProcessor;
    protected boolean mainProcessorStarted = false;

    public CompositePipeline(Function<Mat, Mat> mainProcessor){
        this.mainProcessor = mainProcessor;
    }

    public void addProccessorWithAlias(Consumer<Mat> processor, String alias){
        processingSteps.put(alias, processor);
    }

    public void startProcessor(String alias){
        startedProcessors.add(alias);
    }

    public void stopProcessor(String alias){
        startedProcessors.remove(alias);
    }

    @Override
    public Mat processFrame(Mat input) {
        for(String name : startedProcessors){
            if(processingSteps.containsKey(name)){
                Mat clone = input.clone();
                processingSteps.get(name).accept(clone);
                clone.release();
            }
        }

        if(mainProcessorStarted){
            return mainProcessor.apply(input);
        } else {
            return input;
        }
    }
}
