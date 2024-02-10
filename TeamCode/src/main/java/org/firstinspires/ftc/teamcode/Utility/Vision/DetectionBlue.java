package org.firstinspires.ftc.teamcode.Utility.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware.PropPosition;

public class DetectionBlue {
    static OpenCvWebcam webcam;
    public static YCrCbBlue pipeline;
    static Telemetry telemetry;

    public static void init(HardwareMap hardwareMap, Telemetry telemetry){
        DetectionBlue.telemetry = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new YCrCbBlue();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public static PropPosition getPosition(){
        return pipeline.getLastPosition();
    }
}