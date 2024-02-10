package org.firstinspires.ftc.teamcode.Utility.Vision;

import static org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware.PropPosition.*;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware.PropPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import com.acmerobotics.dashboard.config.Config;

@Config
public class YCrCbBlue extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Scalar rectColor = new Scalar(0, 0, 255);
    Scalar recognizedColor = new Scalar(0, 255, 0);

    Rect leftRect, centerRect, rightRect;

    Mat leftCrop, centerCrop, rightCrop;

    double avgLeft, avgRight, avgCenter;

    public static int heightL = 300;
    public static int widthL = 300;
    public static int heightC = 200;
    public static int widthC = 300;
    public static int heightR = 300;
    public static int widthR = 300;
    public static int leftRectX = 213;
    public static int leftRectY = 200;

    public static int centerRectX = 350;
    public static int centerRectY = 150;

    public static int rightRectX = 426;
    public static int rightRectY = 200;
    private volatile PropPosition position = CENTER;

    public void init(Mat firstFrame){
        leftRect = new Rect(leftRectX - (int)(widthL / 2.0), leftRectY - (int)(heightL / 2.0), widthL, heightL);
        centerRect = new Rect(centerRectX - (int)(widthC / 2.0), centerRectY - (int)(heightC / 2.0), widthC, heightC);
        rightRect = new Rect(rightRectX - (int)(widthR / 2.0), rightRectY - (int)(heightR / 2.0), widthR, heightR);
    }

    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        leftCrop = YCrCb.submat(leftRect);
        centerCrop = YCrCb.submat(centerRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(centerCrop, centerCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        avgLeft = Core.mean(leftCrop).val[0];
        avgCenter = Core.mean(centerCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 4);
        Imgproc.rectangle(output, centerRect, rectColor, 4);
        Imgproc.rectangle(output, rightRect, rectColor, 4);

        double max = Math.max(avgCenter, Math.max(avgRight, avgLeft));

        if(max == avgLeft){
            position = LEFT;
            Imgproc.rectangle(output, leftRect, recognizedColor, 4);
        }else if(max == avgRight){
            position = RIGHT;
            Imgproc.rectangle(output, rightRect, recognizedColor, 4);
        }else{
            position = CENTER;
            Imgproc.rectangle(output, centerRect, recognizedColor, 4);
        }

        return output;
    }

    public PropPosition getLastPosition(){
        return position;
    }

    public String getAverages(){
        return " Left: " + avgLeft + "\n Center: " + avgCenter + "\n Right: " + avgRight;
    }
}