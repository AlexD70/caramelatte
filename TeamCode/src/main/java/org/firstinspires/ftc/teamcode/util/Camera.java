package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import androidx.core.util.Supplier;

import com.pushtorefresh.javac_warning_annotation.Warning;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    protected OpenCvWebcam cam;
    protected OpenCvPipeline pipeline;
    private final String CAMERA_NAME;

    public Camera(HardwareMap hwmap, String name){
        String packageName = hwmap.appContext.getPackageName();
        int id = hwmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", packageName);
        WebcamName camera = hwmap.get(WebcamName.class, name);
        CAMERA_NAME = name;

        cam = OpenCvCameraFactory.getInstance().createWebcam(camera, id);
        cam.setMillisecondsPermissionTimeout(3000);
    }

    public boolean initCamera(OpenCvPipeline pipeline){
        this.pipeline = pipeline;
        final boolean[] cameraOK = {true}; // hate this quick fix so much
        OpenCvCamera.AsyncCameraOpenListener listener = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                cam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                cameraOK[0] = false;
            }
        };

        cam.openCameraDeviceAsync(listener);
        return cameraOK[0];
    }

    public void stopWithoutClosing(){
        cam.stopStreaming();
    }

    public void stopCamera(){
        cam.stopStreaming();
        cam.closeCameraDevice();
    }

    public void changePipeline(OpenCvPipeline pipeline){
        this.pipeline = pipeline;
        cam.setPipeline(pipeline);
    }

    public OpenCvPipeline getPipeline(){
        return pipeline;
    }

    public String getName(){
        return CAMERA_NAME;
    }
}
