package org.firstinspires.ftc.teamcode.Utility.Vision.Prop.New;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.BlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RRBRTeamPropProcessor implements VisionProcessor {
//red right and blue right
    Rect LEFT_RECTANGLE;
    Rect MIDDLE_RECTANGLE;

    Mat hsvMat = new Mat();
    Mat lowMat = new Mat();
    Mat highMat = new Mat();
    Mat detectedMat = new Mat();

    private boolean detectingRed = true;


    double leftThreshold = 0.1;
    double middleThreshold = 0.1;

    private PropPositions previousPropPosition;
    private PropPositions recordedPropPosition = PropPositions.UNFOUND;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        LEFT_RECTANGLE = new Rect(
                new Point(0,0.25 * height),
                new Point(0.45 * width, height)
        );

        MIDDLE_RECTANGLE = new Rect(
                new Point(0.45 * width, 0.25 * height),
                new Point(0.7 * width, height)
        );
    }

    public void setDetectionColor(boolean detectRed) {
        this.detectingRed = detectRed;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        if(this.detectingRed) {
            Scalar lowerRedThresholdLow = new Scalar(0, 125, 125);
            Scalar lowerRedThresholdHigh = new Scalar(20, 255, 255);
            Scalar upperRedThresholdLow = new Scalar(165, 125, 125);
            Scalar upperRedThresholdHigh = new Scalar(180, 255, 255);

            Core.inRange(hsvMat, lowerRedThresholdLow, lowerRedThresholdHigh, lowMat);
            Core.inRange(hsvMat, upperRedThresholdLow, upperRedThresholdHigh, highMat);

            Core.bitwise_or(lowMat, highMat, detectedMat);
        } else {
            Scalar blueHSVLower = new Scalar(85, 100, 20);
            Scalar blueHSVUpper = new Scalar(140, 255, 255);

            Core.inRange(lowMat, blueHSVLower, blueHSVUpper, detectedMat);
        }

        double leftPercent = (Core.sumElems(detectedMat.submat(LEFT_RECTANGLE)).val[0] / 255) / LEFT_RECTANGLE.area();
        double middlePercent = (Core.sumElems(detectedMat.submat(MIDDLE_RECTANGLE)).val[0] / 255) / MIDDLE_RECTANGLE.area();

        Scalar redBorder = new Scalar(255, 0, 0);
        Scalar greenBorder = new Scalar(0, 255, 0);

        PropPositions propPosition;
        if (leftPercent > middlePercent && leftPercent > leftThreshold) {
            //left is highest
            propPosition = PropPositions.LEFT;
            Imgproc.rectangle(frame, LEFT_RECTANGLE, greenBorder);
            Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);

        } else if (middlePercent > leftPercent && middlePercent > middleThreshold) {
            //middle is highest
            propPosition = PropPositions.MIDDLE;

            Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
            Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);

        } else if (middlePercent < middleThreshold && leftPercent < leftThreshold) {
            //right is highest
            propPosition = PropPositions.RIGHT;

            Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
            Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
        } else {
            //team prop not detected
            propPosition = PropPositions.UNFOUND;

            Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
            Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
        }

        if (propPosition != previousPropPosition && propPosition != PropPositions.UNFOUND) {
            recordedPropPosition = propPosition;
        }

        previousPropPosition = propPosition;

        return null;
    }
    public PropPositions getRecordedPropPosition() {
        return recordedPropPosition;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public enum PropPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}